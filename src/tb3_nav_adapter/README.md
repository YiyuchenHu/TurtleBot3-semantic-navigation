# `tb3_nav_adapter`

Selected target → Nav2 goal. A landmark is a point, not a pose, and driving onto it would mean driving into the object — so the goal is offset by a standoff along the robot→object line and yawed to face the object. That standoff is the largest single term in the final distance to the object.

- **Node:** `nav_goal_adapter_node` — Terminal 3, via `tb3_bringup/launch/backend.launch.py`
- **Config:** [`config/nav_goal_adapter.yaml`](config/nav_goal_adapter.yaml) — `approach_distance` (0.5 m), `min_standoff_distance` (0.3 m)
- **Docs:** [README](../../README.md). Known simplification: `compute_approach_pose` treats the target as robot-relative (`base_link`), but map-frame landmarks are passed in, so the standoff is effectively taken along the map-origin → target line. In these small rooms the map origin equals the spawn pose, so the error is modest — except for one hard case: a landmark within about 0.8 m of the map origin (`min_standoff_distance` 0.3 + `approach_distance` 0.5) yields no goal at all, and the coordinator waits in `SEMANTIC_NAV`. The default `warehouse_models_person` world has a figure at (0, 0), which is why `scripts/acceptance_run.sh` defaults to `--world warehouse_models` and reports such a target as `SKIPPED`.
