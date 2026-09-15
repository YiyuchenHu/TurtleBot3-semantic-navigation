"""Runner tests: fake ground_fn + tiny synthetic manifest, no server,
no cv2/numpy — must pass on a bare Mac."""

import json
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from eval import metrics, run_eval  # noqa: E402

GT = [10, 10, 50, 50]
DISTRACTOR = [100, 100, 140, 140]
RANK_HIT = [200, 200, 240, 240]


def _box(bbox, score):
    return {"bbox_xyxy": list(bbox), "score": score, "label": ""}


def fake_ground_fn(image_bytes, query):
    """Deterministic canned answers keyed by query."""
    if query == "find target":
        return [_box(GT, 0.9)]
    if query == "spatial trap":
        # gt-aligned box exists but the top-scoring box is on the
        # distractor -> distractor rejection must fail the case.
        return [_box(DISTRACTOR, 0.9), _box(GT, 0.4)]
    if query == "pick one":
        return [_box(RANK_HIT, 0.8)]
    return []


def _manifest(tmp_path, with_images=True):
    captures = tmp_path / "captures"
    captures.mkdir()
    if with_images:
        for name in ("scene.jpg", "cand_a.jpg", "cand_b.jpg"):
            (captures / name).write_bytes(b"not-a-real-image")
    manifest = {
        "image_root": "captures",
        "world": "warehouse_aws_semantic",
        "notes": "synthetic",
        "cases": [
            {
                "id": "pass_case",
                "mode": "ground",
                "capability": "color",
                "image": "scene.jpg",
                "query": "find target",
                "ground_truth": {"gt_bbox_xyxy": GT},
            },
            {
                "id": "spatial_distractor_fail",
                "mode": "ground",
                "capability": "spatial",
                "image": "scene.jpg",
                "query": "spatial trap",
                "ground_truth": {
                    "gt_bbox_xyxy": GT,
                    "distractor_bboxes": [DISTRACTOR],
                },
            },
            {
                "id": "missing_image",
                "mode": "ground",
                "capability": "color",
                "image": "never_captured.jpg",
                "query": "find target",
                "ground_truth": {"gt_bbox_xyxy": GT},
            },
            {
                "id": "todo_gt",
                "mode": "ground",
                "capability": "identity",
                "image": "scene.jpg",
                "query": "find target",
                "ground_truth": {"gt_bbox_xyxy": "TODO_AFTER_CAPTURE"},
            },
            {
                "id": "rank_pass",
                "mode": "rank",
                "capability": "spatial",
                "query": "pick one",
                "candidates": [
                    {"id": "a", "image": "cand_a.jpg",
                     "ref_bbox_xyxy": RANK_HIT},
                    {"id": "b", "image": "cand_b.jpg",
                     "ref_bbox_xyxy": [0, 0, 40, 40]},
                ],
                "ground_truth": {"expected_winner": "a"},
            },
        ],
    }
    path = tmp_path / "warehouse_test.json"
    path.write_text(json.dumps(manifest))
    return path


def _run(tmp_path, **kwargs):
    path = _manifest(tmp_path)
    manifest = run_eval.load_manifest(path)
    results = run_eval.run_manifest(
        manifest, path.parent, fake_ground_fn, **kwargs)
    return path, results


def _by_id(results):
    return {c["id"]: c for c in results["cases"]}


def test_full_pass_and_verdicts(tmp_path):
    _, results = _run(tmp_path, backend_name="locate_anything")
    cases = _by_id(results)
    assert cases["pass_case"]["verdict"] == "pass"
    assert cases["spatial_distractor_fail"]["verdict"] == "fail"
    assert "distractor" in cases["spatial_distractor_fail"]["reason"]
    assert cases["rank_pass"]["verdict"] == "pass"
    assert cases["rank_pass"]["detail"]["winner"] == "a"


def test_missing_image_skipped_with_capture_hint(tmp_path):
    _, results = _run(tmp_path)
    c = _by_id(results)["missing_image"]
    assert c["verdict"] == "skip"
    assert "capture first" in c["reason"]
    assert "never_captured" in c["reason"]  # tells you which frame to grab


def test_todo_gt_skipped_with_label_hint(tmp_path):
    _, results = _run(tmp_path)
    c = _by_id(results)["todo_gt"]
    assert c["verdict"] == "skip"
    assert "not labeled" in c["reason"]


def test_unsupported_capability_tagged_as_backend_cant(tmp_path):
    # mock backend claims only "color": the spatial distractor failure
    # must be reported as "backend can't", not a plain regression.
    _, results = _run(tmp_path, backend_name="mock")
    cases = _by_id(results)
    assert cases["spatial_distractor_fail"]["verdict"] == "fail_unsupported"
    assert "does not claim" in cases["spatial_distractor_fail"]["reason"]
    # color passes still count normally on mock
    assert cases["pass_case"]["verdict"] == "pass"


def test_only_filter_limits_cases(tmp_path):
    _, results = _run(tmp_path, only="color")
    assert {c["capability"] for c in results["cases"]} == {"color"}
    assert {c["id"] for c in results["cases"]} == {"pass_case",
                                                   "missing_image"}


def test_summary_and_latency(tmp_path):
    _, results = _run(tmp_path, backend_name="locate_anything")
    s = results["summary"]
    assert s["color"] == {"cases": 2, "pass": 1, "fail": 0,
                          "fail_unsupported": 0, "skip": 1, "error": 0,
                          "accuracy": 1.0}
    assert s["spatial"]["pass"] == 1  # rank_pass
    assert s["spatial"]["fail"] == 1  # spatial_distractor_fail
    assert s["spatial"]["accuracy"] == 0.5
    assert s["identity"]["skip"] == 1
    assert s["identity"]["accuracy"] is None
    assert isinstance(results["mean_latency_ms"], float)
    assert results["mean_latency_ms"] >= 0.0


def test_results_json_written_and_schema_sane(tmp_path):
    path, results = _run(tmp_path, backend_name="locate_anything")
    out = path.parent / f"{path.stem}_results.json"
    run_eval.write_results(results, out)
    assert out.is_file()
    loaded = json.loads(out.read_text())
    for key in ("world", "backend", "summary", "cases",
                "mean_latency_ms", "generated_at"):
        assert key in loaded
    assert loaded["world"] == "warehouse_aws_semantic"
    assert loaded["backend"] == "locate_anything"
    for case in loaded["cases"]:
        assert case["verdict"] in ("pass", "fail", "fail_unsupported",
                                   "skip", "error")
        assert {"id", "mode", "capability", "query",
                "reason"} <= set(case)


def test_ground_fn_exception_becomes_error_verdict(tmp_path):
    def exploding(image_bytes, query):
        raise RuntimeError("boom")

    path = _manifest(tmp_path)
    manifest = run_eval.load_manifest(path)
    results = run_eval.run_manifest(manifest, path.parent, exploding)
    cases = _by_id(results)
    assert cases["pass_case"]["verdict"] == "error"
    assert "boom" in cases["pass_case"]["reason"]
    # skips are decided before ground_fn is called
    assert cases["missing_image"]["verdict"] == "skip"


def test_malformed_manifest_raises_manifest_error(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"world": "w", "cases": []}))
    with pytest.raises(run_eval.ManifestError):
        run_eval.load_manifest(bad)
    with pytest.raises(run_eval.ManifestError):
        run_eval.load_manifest(tmp_path / "does_not_exist.json")


def test_format_report_mentions_backend_and_verdicts(tmp_path):
    _, results = _run(tmp_path, backend_name="mock")
    report = run_eval.format_report(results)
    assert "backend: mock" in report
    assert "[PASS] pass_case" in report
    assert "CANT" in report  # unsupported spatial case on mock
