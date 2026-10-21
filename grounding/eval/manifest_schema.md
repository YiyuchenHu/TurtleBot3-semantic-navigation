# Eval manifest schema

A manifest is a single JSON file (see `manifests/warehouse_aws.json`)
describing every eval case for one Gazebo world.

## Top level

| key          | type   | meaning |
|--------------|--------|---------|
| `image_root` | string | Directory containing captured frames, **relative to the manifest file** (e.g. `"../captures"` → `grounding/eval/captures/`). |
| `world`      | string | Gazebo world the frames were captured in (e.g. `"warehouse_aws_semantic"`). Recorded in results for traceability. |
| `notes`      | string | Free-form context: viewpoint descriptions, labeling conventions. |
| `cases`      | array  | Non-empty list of case objects (below). |

## Case object (common fields)

| key          | type   | meaning |
|--------------|--------|---------|
| `id`         | string | Unique per manifest. |
| `mode`       | `"ground"` \| `"rank"` | Scoring mode (below). |
| `query`      | string | The referring expression sent to `POST /ground`. |
| `capability` | `"color"` \| `"identity"` \| `"spatial"` | The *minimum* capability a backend needs to pass this case. Used to keep "backend can't" (e.g. the mock HSV backend on an identity query) apart from "backend got it wrong" in the report. |
| `ground_truth` | object | Mode-specific, see below. May carry a free-form `note`. |

Tag guidance: `color` = pure color attribute ("the blue chair");
`identity` = object class recognition ("pallet jack"); `spatial` =
compositional/spatial relations ("the blue box on top of the orange
sofa"). Tag with the *hardest* capability the case requires.

## `mode: "ground"` — box accuracy in one frame

Extra case fields:

| key     | type   | meaning |
|---------|--------|---------|
| `image` | string | Frame filename relative to `image_root`. |

`ground_truth` fields:

| key                 | type | meaning |
|---------------------|------|---------|
| `gt_bbox_xyxy`      | `[x1, y1, x2, y2]` pixels | The correct object's box in the frame. |
| `distractor_bboxes` | optional array of bboxes | Boxes of *plausible wrong answers* in the same frame (e.g. `box_blue_floor` for "the blue box on top of the sofa"). |

Scoring (`metrics.ground_hit`): **hit** ⇔ some predicted box has
IoU ≥ 0.5 with `gt_bbox_xyxy`, AND (when distractors are listed) the
top-scoring predicted box overlaps the ground truth strictly more than
it overlaps every distractor (distractor rejection).

## `mode: "rank"` — candidate disambiguation (mirrors the ROS resolver)

Extra case fields:

| key          | type  | meaning |
|--------------|-------|---------|
| `candidates` | array | ≥ 2 objects: `{id, image, ref_bbox_xyxy}` — each candidate's evidence frame and its stored box in that frame. Candidates may share a frame with different `ref_bbox_xyxy`. |

`ground_truth` fields:

| key               | type   | meaning |
|-------------------|--------|---------|
| `expected_winner` | string | The candidate `id` that should rank first. |

Scoring: each candidate scores `max(IoU(pred, ref_bbox) × pred.score)`
over the boxes the server returns for its frame; the case passes if the
argmax candidate equals `expected_winner` (top-1). This mirrors
`src/tb3_grounding/tb3_grounding/resolver_core.py`.

## TODO placeholders (before capture/labeling)

Any bbox field may hold the string `"TODO_AFTER_CAPTURE"` instead of a
4-number list. `run_eval.py` treats such cases — and cases whose image
file does not exist yet — as **skipped** with an actionable
"capture first / label first" message; it never crashes on them.
Workflow: capture frames (`capture_frames.py --grab <viewpoint>`),
optionally get YOLO-suggested boxes (`capture_frames.py --annotate
<frame>`), paste pixel coords into the manifest, re-run.
