# Grounding eval harness

Measures how accurately a grounding backend (mock HSV heuristic or the
real NVIDIA LocateAnything-3B behind `grounding/server.py`) recognizes
warehouse objects and resolves compositional/spatial referring
expressions ("the blue box on top of the orange sofa") from camera
frames captured in the `warehouse_aws_semantic` Gazebo world.

Files:

- `manifest_schema.md` — manifest format + metric definitions
- `manifests/warehouse_aws.json` — the warehouse query set (14 cases)
- `metrics.py` — pure-python scoring (IoU, hit logic, rank top-1)
- `run_eval.py` — CLI runner (stdlib only, runs anywhere)
- `capture_frames.py` — in-container frame grabber / labeling helper
- `captures/` — gitignored; your captured frames land here
- `viewpoints.example.json` — named robot viewpoints (fill real coords)

## Workflow

### (a) Launch the sim with the warehouse world

```bash
# in the sim container
ros2 launch tb3_semantic_bringup sim.launch.py world:=warehouse_aws
```

### (b) Move the robot to each viewpoint

Five viewpoints are needed (see `viewpoints.example.json`):
`chairs_front`, `sofa_zone`, `shelf_aisle`, `pallet_jack`, `person`.
Drive with teleop (or teleport the model) until the target objects fill
the camera view. List them any time:

```bash
python3 grounding/eval/capture_frames.py --poses grounding/eval/viewpoints.example.json
```

### (c) Capture frames (in-container)

At each viewpoint:

```bash
python3 grounding/eval/capture_frames.py --grab chairs_front
# -> grounding/eval/captures/chairs_front.jpg
```

Repeat for all five names. (`--topic` defaults to `/camera/image_raw`.)

### (d) Fill in ground-truth boxes

Every `"TODO_AFTER_CAPTURE"` in `manifests/warehouse_aws.json` must
become pixel coordinates `[x1, y1, x2, y2]`. Get suggestions:

```bash
python3 grounding/eval/capture_frames.py --annotate grounding/eval/captures/chairs_front.jpg
```

This prints manual-labeling instructions and, if `ultralytics` is
installed, paste-ready YOLO bbox JSON snippets (verify them — YOLO only
knows COCO classes, so shelf/pallet-jack boxes are manual). Don't
forget the `distractor_bboxes` (e.g. the red chair for "the blue
chair", `box_blue_floor` for the on-the-sofa case) — they power the
distractor-rejection check.

### (e) Run the eval

Against the **mock** backend (CPU, anywhere):

```bash
python3 grounding/server.py --backend mock --port 8801 &
python3 grounding/eval/run_eval.py --manifest grounding/eval/manifests/warehouse_aws.json
```

Against **real LocateAnything** (GPU host):

```bash
python3 grounding/server.py --backend locate_anything --port 8801   # on the GPU box
python3 grounding/eval/run_eval.py \
    --manifest grounding/eval/manifests/warehouse_aws.json \
    --server http://<gpu-host>:8801
```

`--only color|identity|spatial` runs one capability slice.
Exit code: 0 all good, 1 real failures, 2 setup problem (no captures /
server unreachable / bad manifest).

The runner prints a line per case as it goes, so a slow backend looks
slow rather than hung:

```
server: backend=locate_anything  device=cuda  dtype=float16
timeout: 300s per request

[ 1/14] rec_chair              identity  'chair' ... pass  (34.2s)
[ 2/14] rec_box                identity  'box' ... fail  (31.8s)
```

`--timeout` (default 300 s) is the per-request ceiling. **This matters
with the real model:** LocateAnything-3B decodes up to 2048 tokens and
takes tens of seconds per image on a consumer GPU — the model card's
~12.7 boxes/s is an H100 figure. The underlying `GroundingClient`
defaults to 20 s, which is right for live navigation and far too short
here; every case would be scored `ERR` and read as a broken model.
Use `--quiet` to suppress the per-case lines.

## What to expect per backend

The runner tags each case with the minimum capability it needs and
reads the backend name from `/health`, so it can tell "backend can't"
(reported as `CANT`, excluded from accuracy) from "backend got it
wrong" (`FAIL`):

| capability | example case                          | mock (HSV heuristic) | locate_anything |
|------------|---------------------------------------|----------------------|-----------------|
| color      | "the blue chair" vs red-chair distractor | can pass          | should pass     |
| identity   | "pallet jack", "person"               | can't (no object identity) — reported `CANT` | should pass |
| spatial    | "the blue box **on top of** the orange sofa" vs floor distractor | can't (no spatial reasoning) — reported `CANT` | the thing under test |

On mock, a green run means: all color cases `PASS`, identity/spatial
cases `CANT` — that validates the harness plumbing end-to-end without a
GPU.

## (f) Reading the report

```
backend: mock    world: warehouse_aws_semantic    mean latency: 41.2 ms

capability   cases  pass  fail  cant  skip  err  accuracy
--------------------------------------------------------
color            5     5     0     0     0    0      1.00
identity         6     0     0     6     0    0       n/a
spatial          3     0     0     3     0    0       n/a
```

- **accuracy** = pass / (pass + fail), i.e. only over cases the backend
  claims to support.
- **cant** = capability the backend doesn't claim (mock on
  identity/spatial). Expected; not a regression.
- **skip** = frame not captured or gt still `TODO_AFTER_CAPTURE`; the
  per-case line tells you exactly what to do.
- Per-case verdicts follow the table; ground-mode failures say whether
  no box hit IoU ≥ 0.5 or the top box picked a distractor; rank-mode
  failures show each candidate's IoU × score.
- The same report is written as JSON next to the manifest
  (`warehouse_aws_results.json`, gitignored) for diffing across runs.
