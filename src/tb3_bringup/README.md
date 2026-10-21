# `tb3_bringup`

Data-only package: it holds no nodes at all. Every launch file, the RViz config, the Gazebo worlds, the vendored models and the semantic target registry live here, so the six-terminal workflow is `ros2 launch tb3_bringup <something>.launch.py` and nothing else. Each of the five per-terminal launches is standalone — starting one without its upstream neighbours waits rather than crashes.

- **Launches:** `sim` (T1) · `nav` (T2) · `backend` (T3) · `localizer` (T4) · `detector` (T5) · `full_stack` (all five, demo only) · `detector_test` (perception-only world). See [`launch/README.md`](launch/README.md)
- **Config:** [`config/semantic_targets.yaml`](config/semantic_targets.yaml) — the one place `semantic_name` ↔ `detector_label` ↔ `gazebo_model` is defined; every other package reads it from this package's share directory
- **Assets:** [`worlds/`](worlds/) (3 worlds) · [`models/`](models/) (5 vendored Gazebo models, see [NOTICE](../../NOTICE)) · [`rviz/semantic_nav.rviz`](rviz/semantic_nav.rviz)
- **Docs:** [README](../../README.md) · [INSTRUCTIONS](../../INSTRUCTIONS.md) · [NOTES](../../NOTES.md)
