# Frontier detection node: clustering, parameters, testing

## Clustering logic

- **Frontier cells** are free (0) cells with at least one 8-neighbor unknown.
- **Clustering** is by **grid adjacency (8-connected)**:
  - All frontier cells are collected, then we run a **flood-fill (BFS)** over the set of frontier cells.
  - From each unvisited frontier cell, we start a new cluster and recursively add every 8-adjacent frontier cell. That gives one connected component per “blob” of frontier cells.
- **Centroid** per cluster: mean of the **world coordinates** (x, y) of every cell in the cluster (using `grid.info.origin` and `resolution`).
- **Filter:** Clusters with fewer than `min_cluster_size` cells are dropped. The remaining centroids are then filtered against the global costmap; only the accepted ones are published.

So: one centroid per connected frontier blob, after dropping small blobs and unsafe centroids.

---

## Parameter list

Defaults below are the node's built-in defaults; `config/params.yaml` overrides some of them (e.g. `min_cluster_size: 3`).

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_topic` | string | `"/map"` | OccupancyGrid topic (SLAM map). |
| `costmap_topic` | string | `"/global_costmap/costmap"` | Nav2 global costmap used to filter centroids. |
| `frontiers_topic` | string | `"/frontiers"` | PoseArray of safe cluster centroids. |
| `frontiers_markers_topic` | string | `"/frontiers_markers"` | MarkerArray for RViz. |
| `frame_id` | string | `"map"` | Frame for published poses and markers (empty = use the map's `header.frame_id`). |
| `min_cluster_size` | int | `5` | Minimum number of cells per cluster; smaller clusters are discarded. |
| `cost_threshold` | int | `98` | Maximum costmap value (0–100 OccupancyGrid scale) at an accepted centroid. |
| `use_costmap_filter` | bool | `true` | Filter centroids by costmap cost. |
| `costmap_edge_margin` | int | `5` | Reject centroids within this many cells of the costmap border. |
| `publish_raw_markers` | bool | `true` | Also publish raw frontier cells as orange spheres (debug). |
| `publish_rejected_markers` | bool | `true` | Also publish rejected centroids as red spheres (debug). |
| `verbose_frontier_logging` | bool | `false` | Print the step-by-step debug logs at INFO instead of DEBUG. |

See [COSTMAP_FILTER_AND_DEBUG.md](COSTMAP_FILTER_AND_DEBUG.md) for the cost filter and [FRONTIER_DETECTION_DEBUG_LOGS.md](FRONTIER_DETECTION_DEBUG_LOGS.md) for the debug logs.

---

## Build

```bash
cd /path/to/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tb3_frontier_exploration
source install/setup.bash
```

---

## Run

1. Start a map source (e.g. Gazebo + slam_toolbox, or `map_server` with a static map).
2. Run the node:

   ```bash
   ros2 run tb3_frontier_exploration frontier_detection_node
   ```

   Or with config:

   ```bash
   ros2 launch tb3_frontier_exploration frontier_detection.launch.py use_sim_time:=true
   ```

   Override parameters if needed:

   ```bash
   ros2 run tb3_frontier_exploration frontier_detection_node --ros-args \
     -p map_topic:=/map -p min_cluster_size:=5 -p publish_raw_markers:=true
   ```

---

## Test steps in RViz

1. **Fixed Frame:** Set Global Options → Fixed Frame to `map` (or your map frame).
2. **Map:** Add → By topic → `/map` → Map. You should see the occupancy grid.
3. **Frontier centroids (PoseArray):**  
   Add → By topic → `/frontiers` → PoseArray. You should see one pose per accepted cluster centroid.
4. **Markers:**  
   Add → By topic → `/frontiers_markers` → MarkerArray. You should see:
   - **Green spheres** = accepted centroids (namespace `frontier_centroids_safe`), same as the PoseArray.
   - **Red spheres** = centroids rejected by the costmap filter (namespace `frontier_centroids_rejected`), if `publish_rejected_markers` is true.
   - **Orange spheres** = raw frontier cells (namespace `frontier_cells`), if `publish_raw_markers` is true.
5. **Console:** Each map update logs `[frontier] raw=N clusters=M centroids=K safe=S rejected=R` — N raw cells, M clusters, K centroids after `min_cluster_size`, S published and R rejected by the costmap filter.
6. **Tuning:** Increase `min_cluster_size` to reduce small/noisy clusters; decrease to see more centroids. Use the orange markers to verify that clustering matches contiguous frontier regions.

---

## Quick test without a robot

Publish a static map (e.g. `map_server` with a yaml/pgm). The node will update on each `/map` message and publish `/frontiers` (PoseArray) and `/frontiers_markers` (MarkerArray). Without a costmap, centroids are not cost-filtered (all are accepted). Use RViz as above to verify centroids and raw cells.
