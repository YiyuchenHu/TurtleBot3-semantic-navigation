"""NVIDIA LocateAnything-3B backend: image + phrase -> bounding boxes.

Needs an Ampere+ NVIDIA GPU, Linux, ~12 GB VRAM, transformers==4.57.1.
Weights are licensed for non-commercial research use only. Inference
follows the model card at https://huggingface.co/nvidia/LocateAnything-3B.
"""

from __future__ import annotations

import io
import sys
import threading
import time

try:
    from ..boxparse import parse_boxes, parse_labeled_boxes
except ImportError:
    # server.py runs with the grounding dir as sys.path[0], where
    # `backends` is a top-level package and relative import fails.
    from boxparse import parse_boxes, parse_labeled_boxes

_MODEL_ID = "nvidia/LocateAnything-3B"
# Must match the model card's ground_multi template verbatim — the model is
# prompt-sensitive and other phrasings degrade box recall.
_PROMPT_TEMPLATE = ("Locate all the instances that match the following "
                    "description: {query}.")
# Model-card detect() template: category-list detection, categories joined
# with the literal `</c>` separator. Wording verbatim from the model card
# (including "matches"); validated against this checkpoint in R01 §2.3.
_DETECT_TEMPLATE = ("Locate all the instances that matches the following "
                    "description: {categories}.")
_CATEGORY_SEP = "</c>"


def resolve_dtype(name: str, device: str):
    """Pick the compute dtype, defaulting to whatever the GPU can do.

    The model card specifies bfloat16, which needs Ampere (sm_80) or
    newer for *native* support. On Turing (sm_75 — RTX 20xx, T4) bf16
    still runs, but emulated and without tensor cores, so `auto` prefers
    float16 there: Turing accelerates fp16 natively.

    Do NOT use torch.cuda.is_bf16_supported() for this. Through at least
    torch 2.3 it is written as `capability.major >= 8 OR cuda_version >=
    11`, so on a CUDA 12 build it returns True for *every* card —
    including a 2080 Ti, where bf16 is emulated. It answers "will bf16
    run?", not "is bf16 fast?". Compute capability is the honest check.

    The fp16 preference is a real tradeoff, not a free win: bf16 carries
    fp32's exponent range and fp16 does not, so a bf16-trained model can
    overflow to Inf/NaN in fp16. If boxes come back empty or nonsensical
    on a pre-Ampere card, suspect this before suspecting the prompt —
    `--dtype bfloat16` restores the model card's exact numerics at the
    cost of speed, and `--dtype float32` is slower still but safest.
    """
    import torch

    explicit = {
        "bfloat16": torch.bfloat16,
        "float16": torch.float16,
        "float32": torch.float32,
    }
    if name != "auto":
        return explicit[name]

    if device.startswith("cuda") and torch.cuda.is_available():
        try:
            major, _minor = torch.cuda.get_device_capability(device)
        except (RuntimeError, AssertionError, ValueError):
            major = 0
        if major >= 8:          # Ampere and later: native bf16
            return torch.bfloat16
        return torch.float16
    # CPU: fp16 matmuls are largely unimplemented, so fp32 is the only
    # thing that actually runs (slowly).
    return torch.float32


# Generation options and their defaults. Every entry can be set at server
# start (CLI) and overridden per request (`"options": {...}` in the
# /detect and /ground payloads) so the R06a profiler can sweep them
# without restarting the model.
#   generation_mode : "hybrid" (MTP with AR fallback, model-card default),
#                     "fast" (MTP only), "slow" (pure AR)
#   do_sample       : model-card Worker samples (T=0.7, top_p 0.9); False
#                     = greedy: same T/top_p sharpening of the logits, but
#                     argmax instead of a draw — deterministic (R06a).
#                     NOT temperature=0: the remote code's MTP box-frame
#                     validator (is_valid_box_frame / decode_bbox_avg)
#                     tests ABSOLUTE probabilities (0.6/0.7/0.2) that are
#                     calibrated for the sharpened distribution; with T=0
#                     the scaling is skipped, every frame fails the
#                     check, and decoding falls into the AR loop that
#                     emits 341 repeated <box> tokens (6 s/frame,
#                     measured R06a).
#   max_new_tokens  : hard cap on generated tokens (per-request the caller
#                     can tighten it from the category count)
#   max_side        : resize the input so its longer side is <= this many
#                     px before the processor (0 = native). Boxes are
#                     always returned in the ORIGINAL pixel space.
DEFAULT_OPTIONS = dict(generation_mode="hybrid", do_sample=True,
                       temperature=0.7, top_p=0.9, repetition_penalty=1.1,
                       max_new_tokens=2048, max_side=0)


class LocateAnythingBackend:

    def __init__(self, model_id: str = _MODEL_ID, device: str = "cuda",
                 dtype: str = "auto", max_new_tokens: int = 2048,
                 options: dict | None = None, **_kwargs) -> None:
        self.model_id = model_id
        self.device = device
        self.dtype = dtype
        self.options = dict(DEFAULT_OPTIONS)
        self.options["max_new_tokens"] = max_new_tokens
        self.options.update(options or {})
        self._torch_dtype = None   # resolved in load()
        self._model = None
        self._tokenizer = None
        self._processor = None
        self._attn = {}            # attention implementations actually in use
        # Per-forward timing of the language model (R06a profiling): the
        # first forward of a generate() call is the prefill, the rest are
        # decode steps. Filled by a forward hook, read by _generate.
        self._lm_calls: list[float] = []
        self.last_timing: dict = {}
        # One generate at a time: the GPU serializes anyway, the timing
        # hook is per-instance state, and the greedy patch below swaps a
        # module attribute for the duration of a call.
        self._gen_lock = threading.Lock()
        self._generate_utils = None   # remote-code module, resolved lazily

    def load(self) -> None:
        import torch
        from transformers import AutoModel, AutoProcessor, AutoTokenizer

        self._torch_dtype = resolve_dtype(self.dtype, self.device)
        if self.dtype == "auto" and self._torch_dtype is torch.float16:
            cap = torch.cuda.get_device_capability(self.device)
            print(f"[locate_anything] compute capability {cap[0]}.{cap[1]} "
                  "(pre-Ampere): bfloat16 would be emulated, using float16. "
                  "Pass --dtype bfloat16 to match the model card exactly.")

        t0 = time.monotonic()
        self._tokenizer = AutoTokenizer.from_pretrained(
            self.model_id, trust_remote_code=True)
        self._processor = AutoProcessor.from_pretrained(
            self.model_id, trust_remote_code=True)
        self._model = (
            AutoModel.from_pretrained(
                self.model_id,
                torch_dtype=self._torch_dtype,
                trust_remote_code=True,
            )
            .to(self.device)
            .eval()
        )
        if self.device.startswith("cuda") and torch.cuda.is_available():
            reserved = torch.cuda.memory_reserved() / 1024 ** 3
            print(f"[locate_anything] {reserved:.1f} GiB reserved after load")
        # What the remote code actually resolved to (it silently falls
        # back magi -> flash_attention_2 -> sdpa): recorded in /health so
        # every latency number can be attributed.
        cfg = self._model.config
        self._attn = {
            "text_attn": getattr(cfg.text_config, "_attn_implementation", "?"),
            "vision_attn": getattr(cfg.vision_config, "_attn_implementation", "?"),
            "magi_attention": _module_version("magi_attention"),
            "flash_attn": _module_version("flash_attn"),
        }
        self._install_lm_timing_hook()
        print(f"[locate_anything] loaded {self.model_id} on {self.device} "
              f"as {self._torch_dtype} in {time.monotonic() - t0:.1f}s "
              f"attn={self._attn} options={self.options}")
        self._warm_up()

    def _warm_up(self) -> None:
        """One throwaway detect on a synthetic 640x480 frame: the first
        generate after load pays CUDA kernel/allocator warm-up (~600 ms
        vs ~260 ms warm, R06a) and it should not land on a real frame."""
        try:
            from PIL import Image
            buf = io.BytesIO()
            Image.new("RGB", (640, 480), (90, 90, 90)).save(buf, format="JPEG")
            t0 = time.monotonic()
            self.detect(buf.getvalue(), ["person", "traffic cone"])
            print(f"[locate_anything] warm-up detect {1000 * (time.monotonic() - t0):.0f} ms")
        except Exception as exc:  # noqa: BLE001 - warm-up must never block serving
            print(f"[locate_anything] warm-up skipped: {exc}")

    def _install_lm_timing_hook(self) -> None:
        """Time every language-model forward with a CUDA sync at each end;
        the sync costs a few µs per step (the loop is serial anyway)."""
        import torch

        lm = self._model.language_model
        state = {"t0": 0.0}

        def pre(_m, _args, _kwargs):
            if self.device.startswith("cuda"):
                torch.cuda.synchronize()
            state["t0"] = time.perf_counter()

        def post(_m, _args, _kwargs, _out):
            if self.device.startswith("cuda"):
                torch.cuda.synchronize()
            self._lm_calls.append(time.perf_counter() - state["t0"])

        lm.register_forward_pre_hook(pre, with_kwargs=True)
        lm.register_forward_hook(post, with_kwargs=True)

    def info(self) -> dict:
        return {
            "backend": "locate_anything",
            "device": self.device,
            "model": self.model_id,
            # Reported so a run's numbers can be attributed to a dtype
            # later — an fp16 fallback result is not comparable to a bf16
            # one, and /health is the only place the eval can see it.
            "dtype": str(self._torch_dtype).replace("torch.", "")
                     if self._torch_dtype is not None else self.dtype,
            "options": dict(self.options),
            **self._attn,
        }

    def resolve_options(self, override: dict | None) -> dict:
        opts = dict(self.options)
        for k, v in (override or {}).items():
            if k not in DEFAULT_OPTIONS:
                raise ValueError(f"unknown generation option {k!r}")
            opts[k] = type(DEFAULT_OPTIONS[k])(v)
        if opts["generation_mode"] not in ("fast", "slow", "hybrid"):
            raise ValueError(f"bad generation_mode {opts['generation_mode']!r}")
        return opts

    def ground(self, image_bytes: bytes, query: str,
               options: dict | None = None) -> list[dict]:
        answer, width, height = self._generate(
            image_bytes, _PROMPT_TEMPLATE.format(query=query.strip()), options)
        return self._postprocess(parse_boxes, answer, width, height)

    def detect(self, image_bytes: bytes, categories: list[str],
               options: dict | None = None) -> list[dict]:
        """Category-list detection: returns boxes labeled with the `<ref>`
        tag the model grouped them under (raw model text; the server maps
        them back onto the requested categories)."""
        joined = _CATEGORY_SEP.join(c.strip() for c in categories if c.strip())
        answer, width, height = self._generate(
            image_bytes, _DETECT_TEMPLATE.format(categories=joined), options)
        return self._postprocess(parse_labeled_boxes, answer, width, height)

    def _postprocess(self, parser, answer, width, height):
        t0 = time.perf_counter()
        boxes = parser(answer, width, height)
        self.last_timing["postprocess_ms"] = (time.perf_counter() - t0) * 1000.0
        self.last_timing["n_boxes"] = len(boxes)
        return boxes

    def _generate_utils_module(self):
        """The checkpoint's generate_utils (sibling of the model class's
        module in transformers' dynamic-module namespace)."""
        if self._generate_utils is None:
            pkg = type(self._model).__module__.rsplit(".", 1)[0]
            self._generate_utils = sys.modules[pkg + ".generate_utils"]
        return self._generate_utils

    def _generate(self, image_bytes: bytes, question: str,
                  options: dict | None = None):
        with self._gen_lock:
            return self._generate_locked(image_bytes, question, options)

    def _generate_locked(self, image_bytes: bytes, question: str,
                         options: dict | None = None):
        import numpy as np
        import torch
        from PIL import Image

        if self._model is None:
            raise RuntimeError("backend not loaded — call load() first")
        opts = self.resolve_options(options)
        timing: dict = {"options": opts}
        self.last_timing = timing

        # ── preprocess: JPEG decode, optional downscale, patchify ──
        t0 = time.perf_counter()
        image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
        width, height = image.size
        max_side = int(opts["max_side"])
        if max_side > 0 and max(width, height) > max_side:
            scale = max_side / float(max(width, height))
            image = image.resize((max(1, round(width * scale)),
                                  max(1, round(height * scale))),
                                 Image.Resampling.BILINEAR)
        timing["input_wh"] = list(image.size)

        messages = [{"role": "user", "content": [
            {"type": "image", "image": image},
            {"type": "text", "text": question},
        ]}]

        text = self._processor.py_apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True)
        inputs = self._processor(
            text=[text], images=[image], return_tensors="pt"
        ).to(self.device)
        pixel_values = inputs["pixel_values"].to(self._torch_dtype)
        grid_hws = inputs.get("image_grid_hws", None)
        if grid_hws is not None and not torch.is_tensor(grid_hws):
            # generate() does this conversion itself; extract_feature()
            # below is called directly and expects a device tensor.
            grid_hws = torch.from_numpy(np.asarray(grid_hws)).to(
                self.device, dtype=torch.int32)
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()
        t1 = time.perf_counter()
        timing["preprocess_ms"] = (t1 - t0) * 1000.0
        timing["prompt_tokens"] = int(inputs["input_ids"].shape[1])

        with torch.inference_mode():
            # ── vision encoder, run here so it can be timed on its own;
            # generate() accepts the features and only applies the
            # projector (mlp1) to them.
            vit_embeds = self._model.extract_feature(pixel_values, grid_hws)
            if self.device.startswith("cuda"):
                torch.cuda.synchronize()
            t2 = time.perf_counter()
            timing["vision_ms"] = (t2 - t1) * 1000.0
            timing["image_tokens"] = int(sum(int(e.shape[0]) for e in vit_embeds)) \
                if isinstance(vit_embeds, (list, tuple)) else int(vit_embeds.shape[0])

            self._lm_calls.clear()
            gen_kwargs = dict(
                use_cache=True,
                tokenizer=self._tokenizer,
                max_new_tokens=int(opts["max_new_tokens"]),
                generation_mode=opts["generation_mode"],
                repetition_penalty=float(opts["repetition_penalty"]),
            )
            # Same sharpening either way (see DEFAULT_OPTIONS on why T=0
            # is not greedy for this decoder); greedy only swaps the draw
            # for an argmax.
            gen_kwargs.update(temperature=float(opts["temperature"]),
                              do_sample=bool(opts["do_sample"]),
                              top_p=float(opts["top_p"]))
            gu = self._generate_utils_module()
            saved_dists = gu.dists
            if not opts["do_sample"]:
                gu.dists = _ArgmaxDists
            try:
                answer = self._model.generate(
                    pixel_values=pixel_values,
                    input_ids=inputs["input_ids"],
                    attention_mask=inputs["attention_mask"],
                    visual_features=vit_embeds,
                    # Without the processor's patch grid the vision tower
                    # hits grid_hws.tolist() on None.
                    image_grid_hws=grid_hws,
                    **gen_kwargs,
                )
            finally:
                gu.dists = saved_dists
            if self.device.startswith("cuda"):
                torch.cuda.synchronize()
        t3 = time.perf_counter()
        calls = list(self._lm_calls)
        timing["prefill_ms"] = calls[0] * 1000.0 if calls else 0.0
        timing["decode_ms"] = (t3 - t2) * 1000.0 - timing["prefill_ms"]
        timing["decode_forward_ms"] = sum(calls[1:]) * 1000.0
        timing["decode_steps"] = max(0, len(calls) - 1)
        timing["generate_ms"] = (t3 - t2) * 1000.0
        timing["total_ms"] = (t3 - t0) * 1000.0
        ans = answer[0] if isinstance(answer, (list, tuple)) else answer
        timing["answer_len"] = len(ans or "")
        timing["n_box_tokens"] = (ans or "").count("<box>")

        # This revision's generate() returns the decoded string for the
        # new tokens directly (special tokens kept — <box> markers are
        # special tokens); no tensor slicing needed. Boxes are normalized
        # to [0, 1000] so the ORIGINAL width/height maps them back to the
        # caller's pixel space even after a max_side downscale.
        return answer, width, height


class _ArgmaxDists:
    """Stand-in for `torch.distributions` inside the checkpoint's
    generate_utils: `Categorical(probs=p).sample()` returns the argmax, so
    decoding is deterministic while the T/top_p-sharpened probabilities
    still feed the MTP box-frame validator unchanged."""

    class Categorical:
        def __init__(self, probs=None, logits=None):
            self._probs = probs if probs is not None else logits

        def sample(self):
            return self._probs.argmax(dim=-1)


def _module_version(name: str):
    try:
        import importlib
        m = importlib.import_module(name)
    except Exception:
        return None
    return getattr(m, "__version__", "installed")
