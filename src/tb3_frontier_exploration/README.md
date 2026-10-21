# `tb3_frontier_exploration`

Autonomous exploration, in C++. `frontier_detection_node` finds boundaries between mapped and unknown space on the SLAM map; `goal_assignment_node` scores them, sends the winner to Nav2 and retires frontiers that repeatedly fail; `startup_map_warmup_node.py` does the one-shot rotation scan so there is a map to work with. This drives the 2–8 minute exploration phase before any `go to …` can succeed.

- **Nodes:** `frontier_detection_node`, `goal_assignment_node`, `startup_map_warmup_node.py` — Terminal 3, via `tb3_bringup/launch/backend.launch.py`
- **Config:** [`config/params.yaml`](config/params.yaml) — frontier size, goal spacing and blacklist thresholds interlock; NOTES explains how
- **Docs:** [README](../../README.md) · [INSTRUCTIONS](../../INSTRUCTIONS.md) · [NOTES](../../NOTES.md)
