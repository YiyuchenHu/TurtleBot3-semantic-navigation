# `tb3_query`

Stage 4: rule-based command parsing. Turns text on `/user_command` — `go to person`, `go to person 2`, `go to trash can`, `go to chair` — into one object selected from semantic memory, published as `SemanticQueryResult` (defined in `msg/`, which `tb3_coordinator` and `tb3_nav_adapter` build against). No index means the nearest match. No LLM: a whitelist plus a few aliases.

- **Node:** `semantic_query_node` — Terminal 3, via `tb3_bringup/launch/backend.launch.py`
- **Config:** [`config/semantic_query.yaml`](config/semantic_query.yaml) for topics and output frame. The recognised target names come from `tb3_bringup/config/semantic_targets.yaml` — the one place to add or rename a target
- **Docs:** [README](../../README.md) · [INSTRUCTIONS](../../INSTRUCTIONS.md) · [NOTES](../../NOTES.md)
