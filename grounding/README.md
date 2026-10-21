# grounding/ — LocateAnything HTTP server

A minimal, framework-free HTTP wrapper around
[NVIDIA LocateAnything-3B](https://research.nvidia.com/labs/lpr/locate-anything/)
(referring-expression grounding: image + phrase → bounding boxes) that the
ROS 2 stack calls over the network at query time.

This directory is **not a ROS package**: it has no rclpy dependency. In the
normal setup it runs as the `grounding` service in
[`docker/compose.yaml`](../docker/compose.yaml), sharing the sim
container's network namespace so the default `127.0.0.1:8801` resolves
without configuration. It can equally run standalone on a different GPU box
— point `grounding_server_url` in
`src/tb3_query/config/semantic_query.yaml` at it. The ROS side talks to it
through `src/tb3_grounding/tb3_grounding/grounding_client.py`.

## Why a separate server

- Throughput is ~12.7 boxes/s *on an H100* — far too slow to replace
  YOLOv8n per-frame. The pipeline therefore calls it only **at query
  time**, over a handful of stored best-view frames. Keeping it behind
  HTTP makes that boundary structural rather than a convention.
- The model's dependency set is pinned exactly (`transformers==4.57.1`,
  `numpy==1.25.0`, ...) and conflicts with the ROS container's: ROS
  Jazzy's `cv_bridge` needs numpy 1.26 built for Python 3.12, while
  numpy 1.25.0 has no cp312 wheel at all. The two genuinely cannot share
  an interpreter — which is why `Dockerfile.grounding` is on Ubuntu 22.04
  (Python 3.10) while the sim image is on 24.04.
- LocateAnything-3B requires an **NVIDIA Ampere-or-newer GPU, Linux, and
  ~12 GB VRAM** ([model card](https://huggingface.co/nvidia/LocateAnything-3B)).
  Isolating it means the rest of the stack does not inherit that floor.

## API

```
GET  /health
     → {"status": "ok", "backend": "...", "device": "...", "model": "..."}

POST /ground
     {"image": "<base64 jpeg/png>", "query": "sofa with warm color"}
     → {"boxes": [{"bbox_xyxy": [x1, y1, x2, y2], "score": 0.9, "label": "<query>"}, ...],
        "width": 640, "height": 480, "backend": "locate_anything", "latency_ms": 312.4}

POST /detect
     {"image": "<base64 jpeg/png>", "categories": ["person", "bookshelf", ...]}
     → {"detections": [{"bbox_xyxy": [...], "score": 1.0, "label": "person"}, ...],
        "width": 640, "height": 480, "backend": "locate_anything", "latency_ms": 312.4}
```

`/ground` is referring-expression grounding (one free-form phrase);
`/detect` is fixed-vocabulary detection (the model card's category-list
`detect()` template): every returned label is one of the posted category
strings verbatim, and boxes the model tags outside the vocabulary are
dropped server-side. The mapping (`la_detector_node`) uses `/detect`
per frame; query-time search uses `/ground`.

Box coordinates are **pixels** in the posted image. The server is
synchronous and unauthenticated — run it on a trusted network only.

## Backends

### `locate_anything` (the real model)

This is the default backend of the `grounding` compose service — normally
you do not start it by hand:

```bash
docker compose -f docker/compose.yaml up -d grounding
curl -s localhost:8801/health
```

Weights (~7 GB) download from HuggingFace on first start and are cached in
the `hf_cache` named volume; the server does not answer until that
completes.

Standalone, outside the container (Linux + CUDA GPU):

```bash
# Install torch for your CUDA first:
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install "transformers==4.57.1" "numpy==1.25.0" "Pillow==11.1.0" \
            "opencv-python-headless==4.11.0.86" peft "decord==0.6.0" "lmdb==1.7.5"

python3 server.py --backend locate_anything --port 8801
```

Inference follows the model card: prompt
`"Locate all instances matching: <phrase>"`, hybrid generation mode
(parallel box decoding with next-token fallback), output parsed from
`<box><x1><y1><x2><y2></box>` tokens normalized to `[0, 1000]`
(`boxparse.py`).

**License:** the LocateAnything-3B weights permit **non-commercial
research use only**.

### `mock` (GPU-free stand-in)

```bash
# via compose (same image, different backend):
GROUNDING_BACKEND=mock docker compose -f docker/compose.yaml up -d grounding

# or standalone:
pip install opencv-python-headless numpy
python3 server.py --backend mock --port 8801
```

An HSV color heuristic that resolves color-attribute queries ("warm
color", "cool color", "red", "blue", ...) by masking matching pixels and
returning the largest connected regions. It exercises the *entire*
evidence → grounding → IoU-ranking → Nav2 pipeline on a laptop, and is
what the unit tests target. It understands **colors only** — any other
attribute returns no boxes.

## Smoke test

```bash
python3 -m pytest tests/ -q

# with a server running:
curl -s localhost:8801/health
python3 - <<'EOF'
import base64, json, urllib.request
img = base64.b64encode(open("some_frame.jpg", "rb").read()).decode()
req = urllib.request.Request(
    "http://localhost:8801/ground",
    data=json.dumps({"image": img, "query": "sofa with warm color"}).encode(),
    headers={"Content-Type": "application/json"})
print(json.load(urllib.request.urlopen(req)))
EOF
```
