# frontier_detection_node — Design

Code-oriented design for ROS2 Humble. Inputs: `/map` (and optionally `/global_costmap/costmap`). Outputs: `/frontiers`, `/frontiers_markers`.

---

## 1. Node responsibilities

- **Subscribe** to one or two OccupancyGrid sources (map and/or global costmap).
- **Interpret** grid: FREE (0), OCCUPIED (100), UNKNOWN (-1 or 255).
- **Find frontier cells**: free cells that have at least one unknown neighbor (4- or 8-connectivity).
- **Cluster** frontier cells (e.g. by distance or flood-fill).
- **Filter** clusters by minimum size (e.g. min_cells or min_radius_m).
- **Compute** cluster centroid (and optionally size) in world frame.
- **Publish** frontier list and optional RViz markers at a fixed rate or on map update.

---

## 2. Class structure

```cpp
class FrontierDetectionNode : public rclcpp::Node
{
public:
  explicit FrontierDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);  // optional
  void timerCallback();

  // Pipeline
  void updateGridFromMessage(const nav_msgs::msg::OccupancyGrid & msg);
  std::vector<FrontierCluster> findFrontiers();
  geometry_msgs::msg::PoseArray frontiersToPoseArray(const std::vector<FrontierCluster> & clusters);
  visualization_msgs::msg::MarkerArray frontiersToMarkers(const std::vector<FrontierCluster> & clusters);

  // Helpers (see section 4)
  bool isFrontierCell(size_t mx, size_t my) const;
  std::vector<std::pair<int, int>> getNeighborCells(int mx, int my, int connectivity) const;
  std::vector<FrontierCluster> clusterFrontierCells(const std::vector<std::pair<int, int>> & cells);
  std::pair<double, double> clusterCentroid(const std::vector<std::pair<int, int>> & cell_indices) const;
  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;
  void gridToWorld(int gx, int gy, double & wx, double & wy) const;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;  // optional

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontiers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // Timer (if publish by rate instead of on every map update)
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  std::mutex map_mutex_;
  std::vector<FrontierCluster> last_frontiers_;

  // Params (see section 5)
  std::string map_topic_;
  std::string costmap_topic_;
  std::string frame_id_;
  bool use_costmap_;
  double rate_hz_;
  int connectivity_;           // 4 or 8
  int min_cluster_size_;
  double min_frontier_dist_m_;
  bool publish_markers_;
};
```

**FrontierCluster** (internal struct):

```cpp
struct FrontierCluster {
  std::vector<std::pair<int, int>> cells;  // grid indices
  double centroid_x{0.0}, centroid_y{0.0}; // world
  size_t size{0};
};
```

---

## 3. Subscriptions, publishers, timer

| Type         | Name           | Topic / rate              | Purpose |
|--------------|----------------|---------------------------|--------|
| Subscription | `map_sub_`     | `map_topic_` (e.g. `/map`) | Primary grid; updates `current_map_`. |
| Subscription | `costmap_sub_` | `costmap_topic_` (optional) | If `use_costmap_` true, override or fuse with map. |
| Publisher    | `frontiers_pub_` | `/frontiers`            | PoseArray of frontier centroids. |
| Publisher    | `markers_pub_`   | `/frontiers_markers`     | MarkerArray for RViz (spheres/arrows). |
| Timer        | `timer_`       | `1.0 / rate_hz_`          | Periodically recompute and publish (e.g. 1–2 Hz). |

**Data flow choice:**

- **Option A (recommended):** Map callback only stores the latest map (and optionally costmap). Timer at `rate_hz_` runs the pipeline (find → cluster → filter → publish). Keeps publish rate bounded and avoids recomputing on every costmap update.
- **Option B:** Run pipeline inside map/costmap callback and publish immediately. Simpler but can spike CPU if maps update frequently.

Use **Option A** in the design below.

---

## 4. Key helper functions

| Function | Signature (conceptual) | Purpose |
|----------|------------------------|--------|
| `updateGridFromMessage` | `void(const OccupancyGrid &)` | Copy message into `current_map_` (under mutex). |
| `worldToGrid` | `bool(wx, wy, &gx, &gy)` | Map frame → grid index; return false if out of bounds. |
| `gridToWorld` | `void(gx, gy, &wx, &wy)` | Grid index → map frame (cell center). |
| `isFree` | `bool(mx, my)` | Cell is FREE (0). |
| `isUnknown` | `bool(mx, my)` | Cell is UNKNOWN (-1 or 255). |
| `isFrontierCell` | `bool(mx, my)` | Is FREE and has ≥1 UNKNOWN neighbor (4/8). |
| `getNeighborCells` | `vector<pair<int,int>>(mx, my, connectivity)` | Return in-bounds neighbor indices. |
| `findFrontiers` | `vector<FrontierCluster>()` | Scan grid for frontier cells, cluster, filter, compute centroids. |
| `clusterFrontierCells` | `vector<FrontierCluster>(cells)` | Group cells (e.g. flood-fill or distance threshold in grid). |
| `clusterCentroid` | `pair<double,double>(cell_indices)` | Mean (gx, gy) then `gridToWorld`. |
| `frontiersToPoseArray` | `PoseArray(clusters)` | One Pose per cluster (position = centroid, orientation = 0). |
| `frontiersToMarkers` | `MarkerArray(clusters)` | Spheres at centroids; optional IDs and scale. |

**Occupancy conventions:** Use `msg->data[i] == 0` for FREE, `== 100` for OCCUPIED, `== -1` or `>= 99` (or `!= 0 && != 100`) for UNKNOWN depending on map source.

---

## 5. Parameter list

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_topic` | string | `"/map"` | OccupancyGrid topic (SLAM map). |
| `costmap_topic` | string | `"/global_costmap/costmap"` | Optional second grid. |
| `use_costmap` | bool | `false` | If true, use costmap (or fuse) for frontier search. |
| `frame_id` | string | `"map"` | Frame for published poses and markers. |
| `rate` | double | `1.0` | Hz for timer (publish rate). |
| `connectivity` | int | `8` | 4 or 8 for neighbor check. |
| `min_cluster_size` | int | `5` | Minimum cells per cluster to publish. |
| `min_frontier_dist_m` | double | `0.5` | Optional: merge clusters closer than this. |
| `publish_markers` | bool | `true` | Publish `/frontiers_markers`. |
| `unknown_value` | int | `-1` | Grid value for unknown (some use 255). |

---

## 6. Internal data flow

```
  /map (and optionally /global_costmap/costmap)
       │
       ▼
  mapCallback / costmapCallback
       │
       ▼
  updateGridFromMessage()  →  current_map_ (mutex)
       │
       ▼
  timerCallback (at rate_hz_)
       │
       ├─► findFrontiers()
       │        │
       │        ├─► Scan grid: isFrontierCell(mx, my)
       │        ├─► Collect frontier cells
       │        ├─► clusterFrontierCells()  →  clusters
       │        ├─► Filter by min_cluster_size (and optional min_frontier_dist)
       │        └─► clusterCentroid() for each cluster
       │
       ├─► frontiersToPoseArray(clusters)  →  /frontiers
       │
       └─► frontiersToMarkers(clusters)    →  /frontiers_markers (if publish_markers)
```

---

## 7. Recommended message types

| Output | Recommended type | Reason |
|--------|-------------------|--------|
| **/frontiers** | **geometry_msgs/PoseArray** | No new interface; one Pose per frontier (position = centroid); orientation can be 0 or unused. Easy to consume in goal_assignment_node. |
| **/frontiers_markers** | **visualization_msgs/MarkerArray** | Standard for RViz; spheres or arrows at centroids; optional color/size by cluster size. |

**Alternative for /frontiers:** Custom `FrontierArray` with `geometry_msgs/Pose[] poses` and `float32[] sizes` (or `uint32[] cell_counts`) if you need to pass cluster size to the goal selector. For a simple design, **PoseArray is enough**; goal_assignment can use distance or other criteria without size. Prefer **PoseArray + MarkerArray** for simplicity and debuggability.

**Summary:**

- **/frontiers** → `geometry_msgs/msg/PoseArray` (header.frame_id = map, poses = centroids).
- **/frontiers_markers** → `visualization_msgs/msg/MarkerArray` (MARKER_SPHERE or ARROW, ns `"frontiers"`, id by index).

This keeps the node simple, debuggable in RViz, and practical for ROS2 Humble.
