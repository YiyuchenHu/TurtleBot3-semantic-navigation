# Costmap filtering and coordinate conversion

## Class design (extended)

- **Subscriptions:** `/map` (frontier detection), `/global_costmap/costmap` (safety filter).
- **State:** `current_costmap_` (latest costmap, guarded by `costmap_mutex_`). Map is not stored; pipeline runs in map callback.
- **Pipeline (in map callback):** Detect frontier cells from map → cluster → compute centroids (world) → for each centroid, look up cost in costmap → keep only centroids with `cost <= cost_threshold` → publish PoseArray (safe only) and MarkerArray (safe = green, rejected = red, raw cells = orange).
- **Helper:** `getCostAtWorld(wx, wy)` converts world to costmap grid, returns cost or -1 (no costmap / out of bounds).

---

## Map-grid to world-coordinate conversion

Both `/map` and costmap are `nav_msgs/OccupancyGrid`: they have `info.origin` (geometry_msgs/Pose, we use `position.x/y`), `info.resolution` (m/cell), and `info.width`, `info.height`. Data is row-major: index `i = y * width + x`.

**Grid → World (cell center):**
- `wx = origin.position.x + (gx + 0.5) * resolution`
- `wy = origin.position.y + (gy + 0.5) * resolution`
- The `+0.5` places the point at the center of the cell.

**World → Grid (for costmap lookup):**
- `gx = floor((wx - origin.position.x) / resolution)`
- `gy = floor((wy - origin.position.y) / resolution)`
- Clamp `gx` to `[0, width-1]` and `gy` to `[0, height-1]`, then `cost = data[gy * width + gx]`. Costmap data is typically `uint8` (0–255); we read it as unsigned.

Map and costmap usually share the same frame (e.g. `map`). We do **not** use TF here: we assume both grids are in the same world frame, so we use the costmap’s origin and resolution to go from world (cx, cy) to costmap cell and read the cost.

---

## How costmap filtering works

- **Nav2 costmap** values: 0 = FREE, 253 = INSCRIBED, 254 = LETHAL (and gradient in between). Higher cost = closer to obstacles or inside inflation.
- **Parameter `cost_threshold`:** Maximum allowed cost at the centroid. If the cost at (cx, cy) in the costmap is **≤ cost_threshold**, the centroid is **safe** and published. If **> cost_threshold**, it is **rejected** (not in PoseArray; optional red markers for debug).
- **Parameter `use_costmap_filter`:** If `true`, we use the costmap to filter. If costmap has not been received yet, we treat “no cost” as safe and publish all centroids (so the node works without Nav2). Once costmap is available, we filter. If `false`, we never filter by cost.
- **Lookup:** For each centroid world (cx, cy), we convert to costmap grid with the costmap’s origin and resolution, clamp to bounds, and read `data[gy * width + gx]`. Out-of-bounds is treated as unsafe (cost 255) so we reject.

Typical values: `cost_threshold = 0` (only free space), `128` (allow some inflation), `250` (allow almost everything except lethal).

---

## RViz debug recommendations

1. **Fixed Frame:** Set to `map` (or your map frame).
2. **Map:** Add `/map` to see the occupancy grid.
3. **Costmap:** Add `/global_costmap/costmap` to see inflated obstacles (optional; helps verify that rejected centroids lie in high-cost regions).
4. **Frontiers (PoseArray):** Add `/frontiers` — only **safe** centroids; use for goal assignment.
5. **MarkerArray `/frontiers_markers`:**
   - **frontier_centroids_safe** (green): safe centroids, same as PoseArray. Use for “go here” visualization.
   - **frontier_centroids_rejected** (red): centroids rejected by cost. Turn on `publish_rejected_markers: true` to see why some frontiers are not published.
   - **frontier_cells** (orange): raw frontier cells from map (no clustering). Use to debug frontier detection.
6. **Tuning:** If too many frontiers are rejected, increase `cost_threshold`. If the robot goes too close to obstacles, decrease it. Compare green vs red markers over the costmap to confirm filtering is correct.
