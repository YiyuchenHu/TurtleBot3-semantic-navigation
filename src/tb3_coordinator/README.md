# `tb3_coordinator`

Persistent map landmarks and the mode state machine: `semantic_map_memory_node` promotes short-term memory objects into stable landmarks in the `map` frame, and `coordinator_node` arbitrates EXPLORING vs NAVIGATING — pausing frontier exploration while a `/user_command` is served, resuming after.

- **Nodes:** `semantic_map_memory_node`, `coordinator_node` — Terminal 3, via `tb3_bringup/launch/backend.launch.py`. Plus `semantic_runtime_debug_node` (opt-in: `use_runtime_debug:=true`)
- **Config:** [`config/coordinator.yaml`](config/coordinator.yaml)
- **Docs:** [README](../../README.md) · [INSTRUCTIONS](../../INSTRUCTIONS.md) · [NOTES](../../NOTES.md)
