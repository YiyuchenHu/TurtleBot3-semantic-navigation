# `tb3_memory`

Stage 3: short-term semantic memory. Merges repeat sightings of the same physical object into one tracked entry with a stable id (`person_0`, `person_1`, …), so a command can name a specific object rather than whatever is in the current frame. Entries age out when unseen; `tb3_coordinator` promotes the survivors to map landmarks.

- **Node:** `semantic_memory_node` — Terminal 3, via `tb3_bringup/launch/backend.launch.py`
- **Config:** [`config/semantic_memory.yaml`](config/semantic_memory.yaml) — merge distance and lifetime. Merge distance must stay below the smallest real gap between two targets in the world; see NOTES
- **Docs:** [README](../../README.md) · [INSTRUCTIONS](../../INSTRUCTIONS.md) · [NOTES](../../NOTES.md)
