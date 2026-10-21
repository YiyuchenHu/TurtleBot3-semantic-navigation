# grounding/ — LocateAnything HTTP server

A minimal, framework-free HTTP wrapper around
[NVIDIA LocateAnything-3B](https://huggingface.co/nvidia/LocateAnything-3B)
(image + phrase or category list → bounding boxes). It is the only
perception model in the LocateAnything launch path; the ROS 2 stack
reaches it over HTTP.

This directory is **not a ROS package**: `server.py` uses only the Python
standard library plus the backend's own dependencies, no rclpy. In the
normal setup it runs as the `grounding` service in
[`docker/compose.yaml`](../docker/compose.yaml), sharing the `sim`
container's network namespace, so in-container ROS nodes reach it at
`127.0.0.1:8801` without configuration (the port is also published to the
host's loopback). It can equally run standalone on another GPU machine —
point `server_url` in
[`src/tb3_detector/config/la_detector.yaml`](../src/tb3_detector/config/la_detector.yaml)
at it. The ROS-side caller is
[`la_detector_node`](../src/tb3_detector/tb3_detector/la_detector_node.py).

## Why a separate server

- The model's Python dependencies are pinned exactly
  (`transformers==4.57.1`, `numpy==1.25.0`, cu128 `torch==2.11.0`, ...).
  The `sim` image carries a different, CPU-only torch and `numpy==1.26.4`
  for the YOLOv8n baseline and Humble's `cv_bridge`. Keeping the model in
  its own image (`docker/Dockerfile.grounding`, Ubuntu 22.04 / Python
  3.10) avoids mixing the two.
- The GPU requirement stays in one container. With the cu128 wheels the
  host NVIDIA driver must be **>= 570**. The model card targets Ampere or
  newer GPUs; on older cards `--dtype auto` falls back to float16 (see
  `resolve_dtype` in `backends/locate_anything.py`). Measured GPU memory
  of the running server is about **13 GB**.
- Inference is about 0.4 s per `/detect` request with the default
  detector settings (9 categories, 640×480, measured on an RTX 5090), so
  `la_detector_node` samples frames (at most one request per
  `min_interval_sec`, none while the robot turns faster than
  `max_angular_vel`) instead of processing every camera frame.

## API

```
GET  /health
     → {"status": "ok", "backend": "...", "device": "...", "model": "...", ...}

POST /ground
     {"image": "<base64 jpeg/png>", "query": "person wearing a red shirt"}
     → {"boxes": [{"bbox_xyxy": [x1, y1, x2, y2], "score": 1.0, "label": "<query>"}, ...],
        "width": 640, "height": 480, "backend": "locate_anything",
        "latency_ms": 312.4, "timing": {...}}

POST /detect
     {"image": "<base64 jpeg/png>", "categories": ["person", "traffic cone", ...]}
     → {"detections": [{"bbox_xyxy": [...], "score": 1.0, "label": "person"}, ...],
        "width": 640, "height": 480, "backend": "locate_anything",
        "latency_ms": 312.4, "timing": {...}}
```

`/ground` is referring-expression grounding (one free-form phrase; every
box is labelled with the query). `/detect` is fixed-vocabulary detection
(the model card's category-list template): each returned label is one of
the posted category strings verbatim (matched case-insensitively), and
boxes the model tags outside the vocabulary are dropped server-side.

`la_detector_node` uses `/detect` only: on each processed frame it posts
its vocabulary plus coverage categories, and while a search is active
(`/active_search`) the search expression is added to that list. `/ground`
is not used by the ROS stack; it is kept for manual probing.

Both POST routes accept an optional `"options"` object that overrides the
generation options for that request only (`generation_mode`, `do_sample`,
`temperature`, `top_p`, `repetition_penalty`, `max_new_tokens`,
`max_side`, `cap_retry_sample`; see `DEFAULT_OPTIONS` in
`backends/locate_anything.py`). The server-wide defaults come from CLI
flags / environment variables and are reported by `/health`. `timing` is
the per-stage breakdown (vision, prefill, decode) of the request.

Box coordinates are **pixels** in the posted image. The model emits no
per-box confidence, so `score` is always `1.0` for `locate_anything`. The
server is unauthenticated — run it on a trusted network only.

## Backends

### `locate_anything` (the real model)

This is the default backend of the `grounding` compose service — normally
you do not start it by hand. From the repository root:

```bash
docker compose -f docker/compose.yaml -f docker/compose.local.yaml up -d grounding
curl -s localhost:8801/health
```

The weights (`nvidia/LocateAnything-3B`, pinned to revision
`c32291ca5e996f5a7a485845b4f57a233936bba0`, about 7.7 GB) download from
Hugging Face on the first start and are cached in the `hf_cache` named
volume; the server does not answer until loading and a warm-up request
have finished. `HF_HUB_OFFLINE` defaults to `0` so that first download
can happen; once the weights are cached, starting with
`HF_HUB_OFFLINE=1` skips the online check.

Standalone, outside the container (Linux, NVIDIA GPU, driver >= 570,
Python 3.10 or 3.11 — `numpy==1.25.0` has no Python 3.12 wheel):

```bash
pip install --index-url https://download.pytorch.org/whl/cu128 "torch==2.11.0" "torchvision==0.26.0"
pip install "transformers==4.57.1" "numpy==1.25.0" "Pillow==11.1.0" \
            "opencv-python-headless==4.11.0.86" "peft==0.20.0" "decord==0.6.0" \
            "lmdb==1.7.5" "accelerate==1.15.0"

python3 server.py --backend locate_anything --port 8801
```

Inference uses the model card's prompt templates verbatim:
`"Locate all the instances that match the following description: <phrase>."`
for `/ground`, and the category-list form (categories joined with `</c>`)
for `/detect`. Boxes are parsed from `<box><x1><y1><x2><y2></box>` tokens
normalized to `[0, 1000]` (`boxparse.py`).

Generation defaults: the server's own defaults are `hybrid` mode with
sampling (`--generation-mode`, `--sampling`), but the compose service sets
`GROUNDING_GENERATION_MODE=fast` and `GROUNDING_SAMPLING=greedy`, and
`la_detector_node` sends its own per-request options (fast, greedy, a
token cap scaled by the category count). "Greedy" here is an argmax over
the model card's temperature/top-p sharpened distribution, not
`temperature=0`, which breaks this checkpoint's parallel box decoder. If a
greedy request runs into the token cap, the server retries it once with
sampling.

**License:** the LocateAnything-3B weights are licensed by NVIDIA for
**non-commercial research use only**.

### `mock` (GPU-free stand-in)

```bash
# via compose (same image, different backend):
GROUNDING_BACKEND=mock docker compose -f docker/compose.yaml -f docker/compose.local.yaml up -d grounding

# or standalone:
pip install opencv-python-headless numpy
python3 server.py --backend mock --port 8801
```

An HSV color heuristic: it resolves color terms in the query or category
("red", "blue", "warm", "cool", ...) by masking matching pixels and
returning the largest connected regions. It serves the same HTTP contract,
so the server and `la_detector_node` can be exercised without a GPU, and
it is what the unit tests target. It understands **colors only** — a
query or category without a color term returns no boxes, so it cannot
stand in for the real model in navigation runs. The standalone mock needs
no GPU; the compose service still reserves one.

## Smoke test

```bash
cd grounding
python3 -m pytest tests/ -q

# with a server running:
curl -s localhost:8801/health
python3 - <<'EOF'
import base64, json, urllib.request
img = base64.b64encode(open("some_frame.jpg", "rb").read()).decode()
req = urllib.request.Request(
    "http://localhost:8801/detect",
    data=json.dumps({"image": img, "categories": ["person", "traffic cone"]}).encode(),
    headers={"Content-Type": "application/json"})
print(json.load(urllib.request.urlopen(req)))
EOF
```
