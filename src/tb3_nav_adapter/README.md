# `tb3_nav_adapter`

Selected target → Nav2 goal. A landmark is a point, not a pose, and driving onto it would mean driving into the object — so the goal is offset by a standoff along the robot→object line and yawed to face the object. That standoff is the largest single term in the acceptance error budget (NOTES §6.1.1).

- **Node:** `nav_goal_adapter_node` — Terminal 3, via `tb3_bringup/launch/backend.launch.py`
- **Config:** [`config/nav_goal_adapter.yaml`](config/nav_goal_adapter.yaml) — `approach_distance` (0.5 m), `min_standoff_distance` (0.3 m)
- **Docs:** known frame simplification in NOTES §5.1 (bonus fix in §9.2) · [README](../../README.md) · [INSTRUCTIONS](../../INSTRUCTIONS.md)
