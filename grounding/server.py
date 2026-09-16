"""HTTP grounding server (stdlib only).

Exposes a grounding model to the ROS stack over JSON:

    GET  /health
    POST /ground   {"image": "<base64 jpeg/png>", "query": "sofa with warm color"}
        -> {"boxes": [{"bbox_xyxy": [...], "score": f, "label": s}, ...],
            "width": w, "height": h, "backend": str, "latency_ms": f}
    POST /detect   {"image": "<base64 jpeg/png>", "categories": ["person", ...]}
        -> {"detections": [{"bbox_xyxy": [...], "score": f, "label": s}, ...],
            "width": w, "height": h, "backend": str, "latency_ms": f,
            "timing": {...}}
        Fixed-vocabulary detection: every returned label is one of the
        requested category strings, verbatim. Boxes the model groups
        under a tag that matches no requested category are dropped
        (logged), so the caller's vocabulary is authoritative.

    Both POST routes accept an optional "options" object overriding the
    locate_anything generation options for that request only
    (generation_mode / do_sample / temperature / top_p /
    repetition_penalty / max_new_tokens / max_side — see
    backends/locate_anything.py DEFAULT_OPTIONS). The server-wide
    defaults are set by CLI flags and reported by GET /health. "timing"
    is the per-stage breakdown of the last generate (R06a profiling).

Usage:
    python3 server.py --backend locate_anything --port 8801   # GPU box
    python3 server.py --backend mock --port 8801              # CPU demo

Synchronous and unauthenticated; run on a trusted network only.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import json
import os
import struct
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from backends import get_backend

_MAX_BODY_BYTES = 32 * 1024 * 1024


def _image_size(data: bytes) -> tuple[int, int]:
    """Best-effort (width, height) from JPEG/PNG headers; (0, 0) if
    unrecognized. Only used for response metadata."""
    if data[:8] == b"\x89PNG\r\n\x1a\n" and len(data) >= 24:
        w, h = struct.unpack(">II", data[16:24])
        return int(w), int(h)
    if data[:2] == b"\xff\xd8":
        i = 2
        while i + 9 < len(data):
            if data[i] != 0xFF:
                i += 1
                continue
            marker = data[i + 1]
            if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
                h, w = struct.unpack(">HH", data[i + 5:i + 9])
                return int(w), int(h)
            seg_len = struct.unpack(">H", data[i + 2:i + 4])[0]
            i += 2 + seg_len
    return 0, 0


class GroundingHandler(BaseHTTPRequestHandler):
    backend = None  # injected by main() before serving

    def do_GET(self) -> None:  # noqa: N802 (http.server API)
        if self.path.rstrip("/") in ("", "/health"):
            self._send_json(200, {"status": "ok", **self.backend.info()})
        else:
            self._send_json(404, {"error": f"unknown path {self.path}"})

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.rstrip("/")
        if path not in ("/ground", "/detect"):
            self._send_json(404, {"error": f"unknown path {self.path}"})
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > _MAX_BODY_BYTES:
                self._send_json(400, {"error": "missing or oversized body"})
                return
            payload = json.loads(self.rfile.read(length))
            image_b64 = payload["image"]
            if path == "/ground":
                query = str(payload["query"]).strip()
                if not query:
                    raise KeyError("query")
            else:
                categories = [str(c).strip() for c in payload["categories"]
                              if str(c).strip()]
                if not categories:
                    raise KeyError("categories")
            options = payload.get("options") or None
            if options is not None and not isinstance(options, dict):
                raise ValueError("options must be an object")
            image_bytes = base64.b64decode(image_b64, validate=True)
        except (KeyError, TypeError, ValueError, binascii.Error,
                json.JSONDecodeError) as exc:
            self._send_json(400, {"error": f"bad request: {exc}"})
            return

        t0 = time.monotonic()
        try:
            if path == "/ground":
                boxes = self.backend.ground(image_bytes, query, options=options)
                # Referring output carries no usable <ref> label (prompt
                # echo); the query phrase is the label by construction.
                for b in boxes:
                    if not b.get("label"):
                        b["label"] = query
            else:
                boxes = self._detect(image_bytes, categories, options)
        except ValueError as exc:
            self._send_json(400, {"error": f"bad request: {exc}"})
            return
        except Exception as exc:
            # Surface model errors as 500 but keep serving.
            self._send_json(500, {"error": f"grounding failed: {exc}"})
            return
        latency_ms = (time.monotonic() - t0) * 1000.0

        width, height = _image_size(image_bytes)
        self._send_json(200, {
            ("boxes" if path == "/ground" else "detections"): boxes,
            "width": width,
            "height": height,
            "backend": self.backend.info()["backend"],
            "latency_ms": round(latency_ms, 1),
            "timing": getattr(self.backend, "last_timing", {}),
        })

    def _detect(self, image_bytes: bytes, categories: list[str],
                options: dict | None = None) -> list[dict]:
        """Run the backend's detect and enforce the caller's vocabulary:
        each box's raw <ref> label is matched case-insensitively against
        the requested categories and replaced by the matching category
        string verbatim; unmatched labels are dropped and logged."""
        canonical = {c.lower(): c for c in categories}
        kept, dropped = [], []
        for box in self.backend.detect(image_bytes, categories, options=options):
            cat = canonical.get(str(box.get("label", "")).strip().lower())
            if cat is None:
                dropped.append(box.get("label"))
                continue
            box["label"] = cat
            kept.append(box)
        if dropped:
            print(f"[server] /detect dropped {len(dropped)} box(es) with "
                  f"labels outside the requested categories: {dropped!r}")
        return kept

    def _send_json(self, code: int, obj: dict) -> None:
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args) -> None:
        print("[server] %s — %s" % (self.address_string(), fmt % args))


def main() -> None:
    ap = argparse.ArgumentParser(description="Grounding model HTTP server")
    ap.add_argument("--backend", "-b", default="mock",
                    choices=["locate_anything", "mock"])
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", "-p", type=int, default=8801)
    ap.add_argument("--device", default="cuda",
                    help="torch device for locate_anything (default: cuda)")
    ap.add_argument("--dtype", default="auto",
                    choices=["auto", "bfloat16", "float16", "float32"],
                    help="compute dtype for locate_anything. 'auto' uses "
                         "bfloat16 on Ampere+ and float16 on older GPUs "
                         "(Turing has no native bf16).")
    ap.add_argument("--model", default=None,
                    help="override HF model id for locate_anything")
    # locate_anything generation defaults (R06a). Each is also overridable
    # per request through the "options" payload field.
    # Defaults come from the environment so compose can set them without
    # an image rebuild (GROUNDING_GENERATION_MODE / GROUNDING_SAMPLING /
    # GROUNDING_MAX_NEW_TOKENS / GROUNDING_MAX_SIDE).
    env = os.environ.get
    ap.add_argument("--generation-mode",
                    default=env("GROUNDING_GENERATION_MODE", "hybrid"),
                    choices=["fast", "slow", "hybrid"],
                    help="MTP-only / AR-only / MTP with AR fallback (default)")
    ap.add_argument("--sampling", default=env("GROUNDING_SAMPLING", "sample"),
                    choices=["sample", "greedy"],
                    help="model-card sampling (T=0.7, top_p 0.9) or greedy argmax")
    ap.add_argument("--max-new-tokens", type=int,
                    default=int(env("GROUNDING_MAX_NEW_TOKENS", "2048")))
    ap.add_argument("--max-side", type=int, default=int(env("GROUNDING_MAX_SIDE", "0")),
                    help="downscale the input so its longer side is <= N px "
                         "(0 = native); boxes come back in original pixels")
    args = ap.parse_args()

    kwargs = {"device": args.device}
    # dtype is meaningless to the mock backend (pure OpenCV, no torch), so
    # it is not passed there — keeps `--help` honest about what applies.
    if args.backend == "locate_anything":
        kwargs["dtype"] = args.dtype
        kwargs["options"] = dict(generation_mode=args.generation_mode,
                                 do_sample=(args.sampling == "sample"),
                                 max_new_tokens=args.max_new_tokens,
                                 max_side=args.max_side)
    if args.model:
        kwargs["model_id"] = args.model
    backend = get_backend(args.backend, **kwargs)

    print(f"[server] loading backend '{args.backend}' ...")
    backend.load()
    print(f"[server] backend ready: {backend.info()}")

    GroundingHandler.backend = backend
    httpd = ThreadingHTTPServer((args.host, args.port), GroundingHandler)
    print(f"[server] listening on http://{args.host}:{args.port}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[server] shutting down")


if __name__ == "__main__":
    main()
