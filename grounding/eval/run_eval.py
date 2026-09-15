#!/usr/bin/env python3
"""Run the VLM grounding accuracy eval against a manifest.

    python3 grounding/eval/run_eval.py \
        --manifest grounding/eval/manifests/warehouse_aws.json \
        [--server http://127.0.0.1:8801] [--only color|identity|spatial]

For each manifest case the runner loads the captured frame, asks the
grounding server "where is <query>?", scores the answer with
eval/metrics.py, and prints a per-capability accuracy table plus
per-case verdicts. Results are also written as JSON next to the
manifest (<manifest_stem>_results.json).

Backend-agnostic and graceful on the mock backend: cases whose
capability the backend does not claim (see BACKEND_CAPABILITIES) are
reported as "fail (backend can't)" rather than counted as regressions.

stdlib only. The HTTP call reuses the existing GroundingClient
(src/tb3_grounding/tb3_grounding/grounding_client.py, stdlib urllib);
the core loop takes an injectable ground_fn(image_bytes, query) ->
boxes so tests never need a server.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time
from typing import Callable, Optional

try:  # imported as grounding/eval package (tests)
    from . import metrics
except ImportError:  # run as a script: python3 grounding/eval/run_eval.py
    import metrics

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

CAPABILITIES = ("color", "identity", "spatial")

# What each known backend is *designed* to do. A failed case whose
# capability the backend lacks means "backend can't", not "backend got
# it wrong"; the report keeps the two apart. Unknown backends are
# assumed fully capable.
BACKEND_CAPABILITIES = {
    "mock": {"color"},                                   # HSV heuristic
    "locate_anything": {"color", "identity", "spatial"},  # real VLM
}

VERDICT_PASS = "pass"
VERDICT_FAIL = "fail"
VERDICT_FAIL_UNSUPPORTED = "fail_unsupported"  # backend can't do this capability
VERDICT_SKIP = "skip"
VERDICT_ERROR = "error"


class ManifestError(ValueError):
    """Manifest is malformed (not merely un-captured)."""


# --------------------------------------------------------------------------
# manifest loading / validation
# --------------------------------------------------------------------------

def _is_bbox(value) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 4
        and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                for v in value)
    )


def load_manifest(path: pathlib.Path) -> dict:
    if not path.is_file():
        raise ManifestError(f"manifest not found: {path}")
    try:
        manifest = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise ManifestError(f"manifest is not valid JSON ({path}): {exc}")

    for key in ("image_root", "world", "cases"):
        if key not in manifest:
            raise ManifestError(f"manifest missing top-level key '{key}'")
    cases = manifest["cases"]
    if not isinstance(cases, list) or not cases:
        raise ManifestError("manifest 'cases' must be a non-empty list")

    seen_ids = set()
    for case in cases:
        cid = case.get("id")
        if not cid or cid in seen_ids:
            raise ManifestError(f"case id missing or duplicated: {cid!r}")
        seen_ids.add(cid)
        mode = case.get("mode")
        if mode not in ("ground", "rank"):
            raise ManifestError(f"case '{cid}': mode must be ground|rank")
        if case.get("capability") not in CAPABILITIES:
            raise ManifestError(
                f"case '{cid}': capability must be one of {CAPABILITIES}")
        if not case.get("query"):
            raise ManifestError(f"case '{cid}': missing query")
        gt = case.get("ground_truth")
        if not isinstance(gt, dict):
            raise ManifestError(f"case '{cid}': missing ground_truth object")
        if mode == "ground":
            if not case.get("image"):
                raise ManifestError(f"case '{cid}': ground mode needs 'image'")
            if "gt_bbox_xyxy" not in gt:
                raise ManifestError(
                    f"case '{cid}': ground_truth needs gt_bbox_xyxy "
                    "(may be a TODO placeholder until labeled)")
        else:
            cands = case.get("candidates")
            if not isinstance(cands, list) or len(cands) < 2:
                raise ManifestError(
                    f"case '{cid}': rank mode needs >= 2 candidates")
            for cand in cands:
                if not cand.get("id") or not cand.get("image") \
                        or "ref_bbox_xyxy" not in cand:
                    raise ManifestError(
                        f"case '{cid}': each candidate needs "
                        "id, image, ref_bbox_xyxy")
            expected = gt.get("expected_winner")
            if expected not in {c["id"] for c in cands}:
                raise ManifestError(
                    f"case '{cid}': expected_winner must name a candidate")
    return manifest


# --------------------------------------------------------------------------
# core loop (server-free: ground_fn is injected)
# --------------------------------------------------------------------------

def _capture_hint(image_path: pathlib.Path) -> str:
    return (
        f"image not captured: {image_path} — capture first: run "
        f"'python3 grounding/eval/capture_frames.py --grab "
        f"{image_path.stem}' inside the sim container "
        "(see grounding/eval/README.md)"
    )


def _label_hint(case_id: str, field: str) -> str:
    return (
        f"{field} not labeled (TODO placeholder) for case '{case_id}' — "
        "fill pixel coords in the manifest after capture; "
        "'capture_frames.py --annotate <frame>' can suggest boxes"
    )


def run_manifest(
    manifest: dict,
    manifest_dir: pathlib.Path,
    ground_fn: Callable[[bytes, str], list],
    backend_name: str = "unknown",
    only: Optional[str] = None,
    progress: bool = True,
) -> dict:
    """Score every case; returns the results dict (also see write_results).

    ground_fn(image_bytes, query) -> list of {"bbox_xyxy", "score",
    "label"} dicts, matching the /ground response contract.
    """
    image_root = (manifest_dir / manifest["image_root"]).resolve()
    supported = BACKEND_CAPABILITIES.get(backend_name, set(CAPABILITIES))

    case_results = []
    latencies = []

    # Progress is printed per case rather than only in the final report.
    # A real grounding model takes tens of seconds per image on anything
    # short of a datacentre GPU, so a 14-case run is minutes long; with
    # the report as the only output, that is indistinguishable from a
    # hang. Everything is flushed because stdout is a pipe as often as
    # it is a terminal here.
    selected = [c for c in manifest["cases"]
                if not (only and c["capability"] != only)]
    total = len(selected)

    for idx, case in enumerate(selected, 1):
        if progress:
            print(f"[{idx:2d}/{total}] {case['id']:<22} "
                  f"{case['capability']:<9} {case['query'][:44]!r} ... ",
                  end="", flush=True)
        t_case = time.monotonic()
        entry = {
            "id": case["id"],
            "mode": case["mode"],
            "capability": case["capability"],
            "query": case["query"],
            "verdict": None,
            "reason": "",
            "detail": {},
            "latency_ms": None,
        }
        try:
            if case["mode"] == "ground":
                _run_ground_case(case, image_root, ground_fn, entry, latencies)
            else:
                _run_rank_case(case, image_root, ground_fn, entry, latencies)
        except Exception as exc:  # keep evaluating the other cases
            entry["verdict"] = VERDICT_ERROR
            entry["reason"] = f"{type(exc).__name__}: {exc}"

        if entry["verdict"] == VERDICT_FAIL \
                and case["capability"] not in supported:
            entry["verdict"] = VERDICT_FAIL_UNSUPPORTED
            entry["reason"] = (
                f"backend '{backend_name}' does not claim "
                f"'{case['capability']}' capability — expected miss, "
                "not a regression"
            )
        if progress:
            print(f"{entry['verdict']}  ({time.monotonic() - t_case:.1f}s)",
                  flush=True)
            if entry["verdict"] == VERDICT_ERROR:
                # Surfaced immediately: the usual cause is a client
                # timeout, and seeing it on case 1 saves waiting out the
                # remaining thirteen to learn the same thing.
                print(f"          → {entry['reason']}", flush=True)
        case_results.append(entry)

    return {
        "world": manifest.get("world"),
        "backend": backend_name,
        "only": only,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "mean_latency_ms": (
            round(sum(latencies) / len(latencies), 1) if latencies else None
        ),
        "summary": _summarize(case_results),
        "cases": case_results,
    }


def _timed_ground(ground_fn, image_bytes, query, latencies) -> list:
    t0 = time.monotonic()
    boxes = ground_fn(image_bytes, query)
    latencies.append((time.monotonic() - t0) * 1000.0)
    return boxes


def _run_ground_case(case, image_root, ground_fn, entry, latencies) -> None:
    image_path = image_root / case["image"]
    if not image_path.is_file():
        entry["verdict"] = VERDICT_SKIP
        entry["reason"] = _capture_hint(image_path)
        return

    gt = case["ground_truth"]
    gt_bbox = gt["gt_bbox_xyxy"]
    if not _is_bbox(gt_bbox):
        entry["verdict"] = VERDICT_SKIP
        entry["reason"] = _label_hint(case["id"], "gt_bbox_xyxy")
        return
    distractors = gt.get("distractor_bboxes") or []
    if not all(_is_bbox(d) for d in distractors):
        entry["verdict"] = VERDICT_SKIP
        entry["reason"] = _label_hint(case["id"], "distractor_bboxes")
        return

    boxes = _timed_ground(ground_fn, image_path.read_bytes(),
                          case["query"], latencies)
    entry["latency_ms"] = round(latencies[-1], 1)
    detail = metrics.ground_hit(boxes, gt_bbox, distractors or None)
    entry["detail"] = detail
    entry["verdict"] = VERDICT_PASS if detail["hit"] else VERDICT_FAIL
    if not detail["hit"]:
        if not detail["matched"]:
            entry["reason"] = (
                f"no predicted box reached IoU>={metrics.IOU_THRESHOLD} "
                f"vs ground truth (best {detail['best_gt_iou']}, "
                f"{detail['n_boxes']} boxes)"
            )
        else:
            entry["reason"] = (
                "top-scoring box overlapped a distractor at least as much "
                "as the ground truth (distractor not rejected)"
            )


def _run_rank_case(case, image_root, ground_fn, entry, latencies) -> None:
    for cand in case["candidates"]:
        cand_path = image_root / cand["image"]
        if not cand_path.is_file():
            entry["verdict"] = VERDICT_SKIP
            entry["reason"] = _capture_hint(cand_path)
            return
        if not _is_bbox(cand["ref_bbox_xyxy"]):
            entry["verdict"] = VERDICT_SKIP
            entry["reason"] = _label_hint(
                case["id"], f"candidates[{cand['id']}].ref_bbox_xyxy")
            return

    scores = {}
    case_latency = 0.0
    for cand in case["candidates"]:
        boxes = _timed_ground(
            ground_fn, (image_root / cand["image"]).read_bytes(),
            case["query"], latencies)
        case_latency += latencies[-1]
        scores[cand["id"]] = metrics.score_candidate(
            boxes, cand["ref_bbox_xyxy"])

    entry["latency_ms"] = round(case_latency, 1)
    detail = metrics.rank_top1(
        scores, case["ground_truth"]["expected_winner"])
    entry["detail"] = detail
    entry["verdict"] = VERDICT_PASS if detail["correct"] else VERDICT_FAIL
    if not detail["correct"]:
        entry["reason"] = (
            f"expected '{detail['expected']}' to win, got "
            f"{detail['winner']!r} (scores: {detail['scores']})"
        )


def _summarize(case_results: list[dict]) -> dict:
    summary = {}
    for cap in CAPABILITIES:
        rows = [c for c in case_results if c["capability"] == cap]
        if not rows:
            continue
        n = {v: sum(1 for c in rows if c["verdict"] == v)
             for v in (VERDICT_PASS, VERDICT_FAIL, VERDICT_FAIL_UNSUPPORTED,
                       VERDICT_SKIP, VERDICT_ERROR)}
        scored = n[VERDICT_PASS] + n[VERDICT_FAIL]  # excludes "backend can't"
        summary[cap] = {
            "cases": len(rows),
            "pass": n[VERDICT_PASS],
            "fail": n[VERDICT_FAIL],
            "fail_unsupported": n[VERDICT_FAIL_UNSUPPORTED],
            "skip": n[VERDICT_SKIP],
            "error": n[VERDICT_ERROR],
            "accuracy": (
                round(n[VERDICT_PASS] / scored, 3) if scored else None
            ),
        }
    return summary


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------

def format_report(results: dict) -> str:
    lines = []
    lines.append(f"backend: {results['backend']}    "
                 f"world: {results['world']}    "
                 f"mean latency: "
                 f"{results['mean_latency_ms'] or 'n/a'} ms")
    if results.get("only"):
        lines.append(f"filter: --only {results['only']}")
    lines.append("")
    header = (f"{'capability':<12}{'cases':>6}{'pass':>6}{'fail':>6}"
              f"{'cant':>6}{'skip':>6}{'err':>5}{'accuracy':>10}")
    lines.append(header)
    lines.append("-" * len(header))
    for cap, s in results["summary"].items():
        acc = "n/a" if s["accuracy"] is None else f"{s['accuracy']:.2f}"
        lines.append(
            f"{cap:<12}{s['cases']:>6}{s['pass']:>6}{s['fail']:>6}"
            f"{s['fail_unsupported']:>6}{s['skip']:>6}{s['error']:>5}"
            f"{acc:>10}")
    lines.append("")
    lines.append("'cant' = backend does not claim this capability "
                 "(expected misses, excluded from accuracy).")
    lines.append("")
    for c in results["cases"]:
        mark = {
            VERDICT_PASS: "PASS",
            VERDICT_FAIL: "FAIL",
            VERDICT_FAIL_UNSUPPORTED: "CANT",
            VERDICT_SKIP: "SKIP",
            VERDICT_ERROR: "ERR ",
        }[c["verdict"]]
        line = f"  [{mark}] {c['id']:<28} ({c['capability']}, {c['mode']})"
        if c["reason"]:
            line += f"\n         {c['reason']}"
        lines.append(line)
    return "\n".join(lines)


def write_results(results: dict, out_path: pathlib.Path) -> None:
    out_path.write_text(json.dumps(results, indent=2) + "\n")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _make_client(server_url: str, timeout_sec: float = 300.0):
    """Import the existing stdlib GroundingClient from the ROS source
    tree (read-only dependency; it has no ROS or third-party imports).

    The client's own default is 20 s, tuned for the live navigation path
    where a query that slow has already lost the user. Offline eval has
    the opposite priority: LocateAnything-3B decoding up to 2048 tokens
    takes tens of seconds to minutes per image on a consumer GPU (the
    model card's throughput figure is measured on an H100). At 20 s every
    case would time out and be scored ERROR, which reads as a broken
    model rather than an impatient client.
    """
    client_dir = _REPO_ROOT / "src" / "tb3_grounding" / "tb3_grounding"
    if str(client_dir) not in sys.path:
        sys.path.insert(0, str(client_dir))
    from grounding_client import GroundingClient  # noqa: E402
    return GroundingClient(server_url, timeout_sec=timeout_sec)


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Evaluate grounding accuracy against a manifest")
    ap.add_argument("--manifest", required=True, type=pathlib.Path)
    ap.add_argument("--server", default="http://127.0.0.1:8801")
    ap.add_argument("--only", choices=CAPABILITIES, default=None,
                    help="run only cases with this capability tag")
    ap.add_argument("--out", type=pathlib.Path, default=None,
                    help="results JSON path "
                         "(default: <manifest_stem>_results.json "
                         "next to the manifest)")
    ap.add_argument("--timeout", type=float, default=300.0,
                    help="per-request timeout in seconds (default: 300). "
                         "Large models on consumer GPUs need minutes per "
                         "image; the mock backend needs milliseconds.")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress per-case progress lines")
    args = ap.parse_args(argv)

    try:
        manifest = load_manifest(args.manifest)
    except ManifestError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    manifest_dir = args.manifest.resolve().parent
    image_root = (manifest_dir / manifest["image_root"]).resolve()

    # Fresh-clone guard: no point contacting the server before any
    # frame has been captured.
    image_names = set()
    for case in manifest["cases"]:
        if case["mode"] == "ground":
            image_names.add(case["image"])
        else:
            image_names.update(c["image"] for c in case["candidates"])
    if not any((image_root / name).is_file() for name in image_names):
        print(
            f"error: no captured frames found under {image_root}\n"
            "  capture first — inside the sim container run e.g.:\n"
            + "".join(
                f"    python3 grounding/eval/capture_frames.py --grab "
                f"{pathlib.Path(n).stem}\n" for n in sorted(image_names))
            + "  (drive/spawn the robot to each viewpoint first; "
              "see grounding/eval/README.md)",
            file=sys.stderr)
        return 2

    client = _make_client(args.server, timeout_sec=args.timeout)
    health = client.health()
    if health is None:
        print(
            f"error: grounding server unreachable at {args.server}\n"
            "  start one with: python3 grounding/server.py --backend mock "
            "--port 8801\n"
            "  (or --backend locate_anything on a GPU host)",
            file=sys.stderr)
        return 2
    backend_name = health.get("backend", "unknown")

    if not args.quiet:
        # Echo what /health reported. dtype in particular decides whether
        # a run is comparable to another one (fp16 vs the model card's
        # bf16), and it is invisible from this side otherwise.
        detail = "  ".join(f"{k}={v}" for k, v in health.items()
                           if k in ("backend", "device", "dtype", "model"))
        print(f"server: {detail}\ntimeout: {args.timeout:.0f}s per request\n")

    results = run_manifest(manifest, manifest_dir, client.ground,
                           backend_name=backend_name, only=args.only,
                           progress=not args.quiet)
    results["server"] = args.server
    results["manifest"] = str(args.manifest)

    print(format_report(results))
    out_path = args.out or (
        manifest_dir / f"{args.manifest.stem}_results.json")
    write_results(results, out_path)
    print(f"\nresults written to {out_path}")

    any_fail = any(c["verdict"] in (VERDICT_FAIL, VERDICT_ERROR)
                   for c in results["cases"])
    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
