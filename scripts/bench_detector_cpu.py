#!/usr/bin/env python3
"""CPU inference cost of the course detector weights, measured the way the
detector node runs them: ultralytics predict() on 640x480 BGR frames, one frame
at a time, on the CPU, in this environment (no conda, ultralytics 8.4.31).

    python3 scripts/bench_detector_cpu.py --frames <dir with .jpg> [--n 200]

Every listed weights file is run on the SAME frames, in the same process, after a
warm-up, and the fine-tuned model is run again at the end so thermal drift shows
up as a difference between its two rows.  Reports per-frame milliseconds.
"""
import argparse
import glob
import os
import statistics
import time

import cv2
import numpy as np


def bench(weights, frames, conf, warmup=20, track=False):
    """track=True times model.track(persist=True) -- ByteTrack on top of the same
    inference -- which is what the node runs with enable_tracking: true."""
    from ultralytics import YOLO
    m = YOLO(weights)
    run = (lambda im: m.track(im, conf=conf, device="cpu", persist=True, verbose=False)) if track \
        else (lambda im: m.predict(im, conf=conf, device="cpu", verbose=False))
    for im in frames[:warmup]:
        run(im)
    ms, ndet = [], 0
    for im in frames:
        t0 = time.perf_counter()
        r = run(im)[0]
        ms.append((time.perf_counter() - t0) * 1000.0)
        ndet += len(r.boxes)
    return ms, ndet


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True)
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--weights", nargs="+", default=[
        ("src/tb3_detector/models/tb3det_yolo26n.pt", 0.40),
        ("src/tb3_detector/models/yolo26n.pt", 0.35),
        ("src/tb3_detector/models/tb3det_yolo26n.pt", 0.40)])
    ap.add_argument("--track", action="store_true",
                    help="also time the fine-tuned weights with ByteTrack (model.track) on the same "
                         "frames, to measure the per-frame cost of enable_tracking: true")
    ap.add_argument("--ordered", action="store_true",
                    help="keep the frames in filename order instead of striding through the set; "
                         "use with --track so the tracker sees consecutive views")
    a = ap.parse_args()
    import torch
    paths = sorted(glob.glob(os.path.join(a.frames, "*.jpg")))
    step = 1 if a.ordered else max(1, len(paths) // a.n)
    paths = paths[::step][: a.n]
    frames = [cv2.imread(p) for p in paths]
    assert all(f is not None and f.shape == (480, 640, 3) for f in frames), "expect 640x480 BGR frames"
    cpu = os.popen("lscpu | grep 'Model name' | sed 's/.*: *//'").read().strip()
    print(f"{len(frames)} frames from {a.frames}; torch {torch.__version__}, "
          f"threads={torch.get_num_threads()}, cpu={cpu}")
    rows = []
    for item in a.weights:
        w, conf = (item if isinstance(item, tuple) else (item, 0.40))
        ms, ndet = bench(w, frames, conf)
        rows.append((os.path.basename(w), conf, statistics.median(ms), float(np.percentile(ms, 95)),
                     statistics.mean(ms), ndet / len(frames)))
    if a.track:
        w, conf = "src/tb3_detector/models/tb3det_yolo26n.pt", 0.40
        for label, tr in (("+ ByteTrack", True), ("(predict again)", False)):
            ms, ndet = bench(w, frames, conf, track=tr)
            rows.append((os.path.basename(w) + " " + label, conf, statistics.median(ms),
                         float(np.percentile(ms, 95)), statistics.mean(ms), ndet / len(frames)))
    print(f"\n{'weights':36s} {'conf':>5s} {'median ms':>10s} {'p95 ms':>8s} {'mean ms':>8s} {'det/frame':>10s}")
    for r in rows:
        print(f"{r[0]:36s} {r[1]:5.2f} {r[2]:10.1f} {r[3]:8.1f} {r[4]:8.1f} {r[5]:10.2f}")


if __name__ == "__main__":
    main()
