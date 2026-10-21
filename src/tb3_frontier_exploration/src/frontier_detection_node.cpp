#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <set>
#include <queue>
#include <mutex>
#include <cstdint>

/**
 * @file frontier_detection_node.cpp
 * @brief ROS 2 node: detect exploration frontiers from SLAM map and filter with global costmap.
 *
 * This node is the bridge between mapping (occupancy information) and downstream goal assignment:
 *
 * - Subscribes to the SLAM / mapping occupancy grid (typically `/map`) and the Nav2 global costmap
 *   (`/global_costmap/costmap` by default). Both are `nav_msgs/OccupancyGrid` messages.
 * - Detects **frontier cells**: free cells in the map that are 8-connected to at least one **unknown**
 *   cell (the boundary between explored free space and unexplored territory).
 * - Groups frontier cells into connected components via 8-neighbor adjacency, then computes each
 *   cluster’s centroid in **map/world** coordinates as a compact exploration candidate.
 * - Optionally filters each centroid using the global costmap (rejecting high-cost or out-of-map poses)
 *   so only **safe** candidates are published.
 * - Publishes accepted centroids as `geometry_msgs/PoseArray` for a goal-assignment or navigation node.
 * - Publishes `visualization_msgs/MarkerArray` for RViz (raw frontier cells, safe centroids, and
 *   optionally rejected centroids).
 *
 * Downstream nodes consume the frontier poses; this node does not send Nav2 goals itself.
 */

namespace tb3_frontier_exploration
{

using Cell = std::pair<int, int>;
using Cluster = std::vector<Cell>;

class FrontierDetectionNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the frontier detection node, declare parameters, and wire subscribers/publishers.
   *
   * @param none Constructor takes no arguments; behavior is configured via ROS parameters.
   * @return N/A (constructs the node in place).
   *
   * Pipeline role:
   * - Instantiates the sensing side of frontier exploration: map + costmap inputs and frontier/marker
   *   outputs. Without this setup, callbacks would not run and nothing would be published.
   *
   * Implementation summary:
   * 1. Declares parameters for topic names, clustering (`min_cluster_size`), cost filtering
   *    (`cost_threshold`, `use_costmap_filter`), and visualization flags.
   * 2. Creates a subscription to the occupancy grid map and one to the global costmap.
   * 3. Creates publishers for `PoseArray` frontiers and `MarkerArray` visualization.
   * 4. Logs resolved topic names at startup.
   *
   * Notes:
   * - Parameters must match your TF/map frame setup (`frame_id` overrides published message frames when set).
   * - Costmap subscription is asynchronous; until the first costmap arrives, cost lookups may return -1
   *   and filtering behavior follows `use_costmap_filter` (see `mapCallback`).
   */
  FrontierDetectionNode()
  : Node("frontier_detection_node")
  {
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    declare_parameter<std::string>("frontiers_topic", "/frontiers");
    declare_parameter<std::string>("frontiers_markers_topic", "/frontiers_markers");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<int>("min_cluster_size", 5);
    // Compared against /global_costmap/costmap, an OccupancyGrid whose known
    // cells only ever carry 0..100 (Nav2 translates raw 253 -> 99 inscribed,
    // 254 -> 100 lethal, 255 -> -1 unknown). 98 therefore rejects exactly the
    // inscribed and lethal cells and nothing else. Anything above 100 disables
    // the cost test entirely, which is what the previous 128/150 defaults did.
    declare_parameter<int>("cost_threshold", 98);
    declare_parameter<bool>("use_costmap_filter", true);
    declare_parameter<bool>("publish_raw_markers", true);
    declare_parameter<bool>("publish_rejected_markers", true);
    declare_parameter<bool>("verbose_frontier_logging", false);
    declare_parameter<int>("costmap_edge_margin", 5);

    std::string map_topic = get_parameter("map_topic").as_string();
    std::string costmap_topic = get_parameter("costmap_topic").as_string();
    std::string frontiers_topic = get_parameter("frontiers_topic").as_string();
    std::string markers_topic = get_parameter("frontiers_markers_topic").as_string();

    // Both /map (slam_toolbox) and /global_costmap/costmap (Nav2) are published
    // TRANSIENT_LOCAL. Subscribing VOLATILE still connects, but drops the
    // latched frame that is already on the wire at connect time, so this node
    // stayed blind until the next periodic republish. goal_assignment_node
    // already subscribes to the costmap this way; match it here.
    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, latched_qos, std::bind(&FrontierDetectionNode::mapCallback, this, std::placeholders::_1));
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, latched_qos,
      std::bind(&FrontierDetectionNode::costmapCallback, this, std::placeholders::_1));

    frontiers_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(frontiers_topic, 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic, 10);

    RCLCPP_INFO(get_logger(), "frontier_detection: map=%s, costmap=%s, frontiers=%s",
      map_topic.c_str(), costmap_topic.c_str(), frontiers_topic.c_str());
  }

private:
  static constexpr int8_t FREE = 0;
  static constexpr int8_t OCCUPIED = 100;

  /**
   * @brief Test whether an occupancy grid cell value represents traversable free space per this node.
   *
   * @param value Single cell from `OccupancyGrid.data` (0–100 per `nav_msgs/OccupancyGrid` convention,
   *              or -1 for unknown when used elsewhere; here only exact 0 counts as free).
   * @return `true` if the cell is treated as free (`value == 0`).
   *
   * Pipeline role:
   * - Defines the “free” side of the free/unknown frontier test used by `isFrontierCell`.
   *
   * Implementation summary:
   * 1. Compares the byte to the constant `FREE` (0).
   *
   * Notes:
   * - Values in (0, 100] are **not** free here; only strict 0 is free. This matches common SLAM maps
   *   where 0 is free and 100 is occupied.
   */
  static bool isFree(int8_t value) { return value == FREE; }

  /**
   * @brief Test whether an occupancy value is treated as unknown (or otherwise non-standard) for frontier detection.
   *
   * @param value Single cell from `OccupancyGrid.data`.
   * @return `true` if the value is outside the inclusive range [0, 100] (e.g. -1 unknown, or invalid >100).
   *
   * Pipeline role:
   * - Defines the “unknown” side of the frontier predicate: a frontier requires a free cell adjacent to
   *   a cell classified as unknown by this rule.
   *
   * Implementation summary:
   * 1. Returns true when `value < 0` or `value > OCCUPIED` (100).
   *
   * Notes:
   * - Partially occupied or cost-like interpretations in (0,100] are **not** unknown here—only
   *   strictly outside [0,100]. Aligns with typical `nav_msgs` maps where -1 means unknown.
   */
  static bool isUnknown(int8_t value) { return value < 0 || value > OCCUPIED; }

  /**
   * @brief Decide if map cell (mx, my) is a frontier: free and 8-adjacent to unknown.
   *
   * @param grid Occupancy grid (usually the SLAM `/map`) with valid `info` and `data`.
   * @param mx Column index in map coordinates (0 .. width-1).
   * @param my Row index in map coordinates (0 .. height-1).
   * @return `true` if the cell is free and has at least one 8-neighbor that is unknown (per `isUnknown`).
   *
   * Pipeline role:
   * - Core geometric predicate for frontier exploration: identifies cells on the exploration boundary.
   *
   * Implementation summary:
   * 1. Bounds-check (mx, my); map linear index = `my * width + mx`.
   * 2. If the cell is not free, return false.
   * 3. Scan 8 neighbors; if any in-bounds neighbor is unknown, return true.
   * 4. Otherwise return false.
   *
   * Notes:
   * - Out-of-bounds neighbors are skipped (treated as not unknown), not as frontier triggers.
   * - Uses 8-connectivity (including diagonals), so frontiers can be thicker along diagonal unknown regions.
   */
  bool isFrontierCell(const nav_msgs::msg::OccupancyGrid & grid, int mx, int my) const
  {
    const int w = static_cast<int>(grid.info.width);
    const int h = static_cast<int>(grid.info.height);
    if (mx < 0 || mx >= w || my < 0 || my >= h) return false;
    const size_t idx = static_cast<size_t>(my) * grid.info.width + static_cast<size_t>(mx);
    if (!isFree(grid.data[idx])) return false;
    const int dx[] = {-1, -1, -1,  0,  0,  1,  1,  1};
    const int dy[] = {-1,  0,  1, -1,  1, -1,  0,  1};
    for (int i = 0; i < 8; i++) {
      int nx = mx + dx[i], ny = my + dy[i];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      size_t nidx = static_cast<size_t>(ny) * grid.info.width + static_cast<size_t>(nx);
      if (isUnknown(grid.data[nidx])) return true;
    }
    return false;
  }

  /**
   * @brief Convert integer grid indices to world (map) frame coordinates at cell center.
   *
   * @param grid Grid whose `info.origin` and `info.resolution` define the map->world transform.
   * @param gx Grid column index (same convention as map x-index).
   * @param gy Grid row index (same convention as map y-index).
   * @param wx Output: world x (meters) of the cell center.
   * @param wy Output: world y (meters) of the cell center.
   * @return void; results written to `wx` and `wy`.
   *
   * Pipeline role:
   * - Turns clustered frontier indices into poses for publishing and for costmap sampling at centroids.
   *
   * Implementation summary:
   * 1. `wx = origin.x + (gx + 0.5) * resolution` (cell center).
   * 2. `wy = origin.y + (gy + 0.5) * resolution`.
   *
   * Notes:
   * - Assumes 2D map (z ignored); consistent with `PoseArray` publishing z=0.
   * - Uses +0.5 offset so the representative point is the cell center, not the corner.
   */
  void gridToWorld(const nav_msgs::msg::OccupancyGrid & grid, int gx, int gy,
                   double & wx, double & wy) const
  {
    wx = grid.info.origin.position.x + (static_cast<double>(gx) + 0.5) * grid.info.resolution;
    wy = grid.info.origin.position.y + (static_cast<double>(gy) + 0.5) * grid.info.resolution;
  }

  /**
   * @brief Partition frontier cells into maximal 8-connected components (clusters).
   *
   * @param cells List of all frontier cell coordinates `(mx, my)` to group.
   * @param width Map width in cells (used for neighbor bounds checks).
   * @param height Map height in cells (used for neighbor bounds checks).
   * @return Vector of clusters; each cluster is a list of cells connected via 8-neighborhood within `cells`.
   *
   * Pipeline role:
   * - Reduces thousands of frontier pixels to a small set of regions; each region later becomes one
   *   centroid candidate for exploration.
   *
   * Implementation summary:
   * 1. Insert all `cells` into a `std::set` for O(log n) membership tests.
   * 2. For each unvisited seed cell, run BFS/queue flood fill over 8-neighbors that remain in the set.
   * 3. Every discovered component is pushed to the output vector.
   *
   * Notes:
   * - Only adjacency **among cells in `cells`** matters; a frontier pixel cannot link through a non-frontier cell.
   * - Empty input yields empty output; single-cell frontiers form singleton clusters.
   */
  std::vector<Cluster> clusterByAdjacency(
    const std::vector<Cell> & cells,
    int width, int height) const
  {
    std::set<Cell> frontier_set(cells.begin(), cells.end());
    std::set<Cell> visited;
    std::vector<Cluster> clusters;
    const int dx[] = {-1, -1, -1,  0,  0,  1,  1,  1};
    const int dy[] = {-1,  0,  1, -1,  1, -1,  0,  1};

    for (const Cell & seed : cells) {
      if (visited.count(seed)) continue;
      Cluster cluster;
      std::queue<Cell> q;
      q.push(seed);
      visited.insert(seed);
      while (!q.empty()) {
        Cell c = q.front();
        q.pop();
        cluster.push_back(c);
        for (int i = 0; i < 8; i++) {
          Cell n(c.first + dx[i], c.second + dy[i]);
          if (n.first < 0 || n.first >= width || n.second < 0 || n.second >= height) continue;
          if (frontier_set.count(n) && !visited.count(n)) {
            visited.insert(n);
            q.push(n);
          }
        }
      }
      clusters.push_back(std::move(cluster));
    }
    return clusters;
  }

  /**
   * @brief Compute the Euclidean centroid of a cluster in world coordinates (arithmetic mean of cell centers).
   *
   * @param grid Map grid used only for `gridToWorld` (same frame as published poses).
   * @param cluster One connected component of frontier cells.
   * @param cx Output: mean world x of cluster cell centers.
   * @param cy Output: mean world y of cluster cell centers.
   * @return void; if `cluster` is empty, leaves `cx`/`cy` at 0.0.
   *
   * Pipeline role:
   * - Produces a single representative point per frontier blob for cost filtering and goal proposal.
   *
   * Implementation summary:
   * 1. Sum world (x,y) for each cell via `gridToWorld`.
   * 2. Divide by cluster size.
   *
   * Notes:
   * - Centroid may fall on a non-free cell geometrically; costmap filtering mitigates unsafe proposals.
   * - This is a simple mean, not a geometric “pole of inaccessibility” or skeleton point.
   */
  void clusterCentroid(const nav_msgs::msg::OccupancyGrid & grid,
                       const Cluster & cluster, double & cx, double & cy) const
  {
    cx = 0.0;
    cy = 0.0;
    if (cluster.empty()) return;
    for (const Cell & c : cluster) {
      double wx, wy;
      gridToWorld(grid, c.first, c.second, wx, wy);
      cx += wx;
      cy += wy;
    }
    cx /= static_cast<double>(cluster.size());
    cy /= static_cast<double>(cluster.size());
  }

  /**
   * @brief Look up global costmap cost at a world position (discarding grid indices).
   *
   * @param wx World x (meters), same frame as costmap `header.frame_id` / map frame.
   * @param wy World y (meters).
   * @return 0..100 for a known cell (the OccupancyGrid scale Nav2 publishes), 255 for a cell the costmap
   *   marks unknown (grid -1, which the unsigned-char cast below turns into 255), 255 if out of bounds,
   *   or -1 if no costmap has arrived yet. Note the two different 255s and the single -1: only "no costmap
   *   at all" reaches the caller as a negative value.
   *
   * Pipeline role:
   * - Thin wrapper for callers that only need the cost value, not the discretized map coordinates.
   *
   * Implementation summary:
   * 1. Delegates to `getCostAtWorldWithIndices` with dummy index outputs.
   *
   * Notes:
   * - Locks the costmap mutex indirectly via the callee; safe to call from `mapCallback` thread.
   */
  int getCostAtWorld(double wx, double wy) const
  {
    int mx = 0, my = 0;
    return getCostAtWorldWithIndices(wx, wy, mx, my);
  }

  /**
   * @brief Sample the latest global costmap at world coordinates and expose the grid indices used.
   *cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tb3_frontier_exploration exploration.launch.py use_sim_time:=true
   * @param wx World x (meters) in the costmap’s frame.
   * @param wy World y (meters) in the costmap’s frame.
   * @param out_mx Output: computed grid column before clamping (may be outside [0,width) if OOB).
   * @param out_my Output: computed grid row before clamping (may be outside [0,height) if OOB).
   * @return Occupancy/cost byte as `int` (0–253 typical for costmaps), **255** if (wx,wy) maps outside
   *         the grid, or **-1** if no costmap message has been received yet.
   *
   * Pipeline role:
   * - Implements the safety gate between “interesting frontier” and “publishable goal”: high cost or OOB
   *   means the centroid is rejected when filtering is enabled.
   *
   * Implementation summary:
   * 1. Lock `costmap_mutex_` for thread-safe read of `current_costmap_`.
   * 2. If no costmap, return -1.
   * 3. Convert world to grid with `floor((w - origin) / resolution)` and write indices to outputs.
   * 4. If indices out of range, return 255; else return `data[gy * width + gx]` as unsigned char cast to int.
   *
   * Notes:
   * - Assumes costmap `data` is row-major matching `nav_msgs/OccupancyGrid` layout.
   * - World and costmap must be aligned (same global frame) for meaningful costs; TF is not handled here.
   * - Return -1 means “no costmap received yet” only, and `mapCallback` treats it as accept (see the
   *   `use_costmap_filter` branch with `cost < 0`). A cell the costmap itself marks unknown does NOT come
   *   back negative: the `unsigned char` cast maps grid -1 to 255, which is above any `cost_threshold` in
   *   the 0..100 range and is therefore rejected. That unknown/out-of-bounds rejection is the only part of
   *   this filter that was ever active while `cost_threshold` sat on the raw 0-255 scale.
   */
  int getCostAtWorldWithIndices(double wx, double wy, int & out_mx, int & out_my) const
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!current_costmap_) return -1;
    const auto & cm = *current_costmap_;
    const double ox = cm.info.origin.position.x;
    const double oy = cm.info.origin.position.y;
    const double res = cm.info.resolution;
    const int w = static_cast<int>(cm.info.width);
    const int h = static_cast<int>(cm.info.height);
    int gx = static_cast<int>(std::floor((wx - ox) / res));
    int gy = static_cast<int>(std::floor((wy - oy) / res));
    out_mx = gx;
    out_my = gy;
    if (gx < 0 || gx >= w || gy < 0 || gy >= h) return 255;
    size_t idx = static_cast<size_t>(gy) * cm.info.width + static_cast<size_t>(gx);
    return static_cast<int>(static_cast<unsigned char>(cm.data[idx]));
  }

  /**
   * @brief Store the latest global costmap message for thread-safe lookups from `mapCallback`.
   *
   * @param msg Incoming `nav_msgs/OccupancyGrid` from the global costmap topic.
   * @return void.
   *
   * Pipeline role:
   * - Keeps Nav2’s inflated/obstacle representation available so frontier centroids can be filtered without
   *   recomputing costs from scratch.
   *
   * Implementation summary:
   * 1. Ignore zero-sized grids.
   * 2. Under `costmap_mutex_`, assign `current_costmap_ = msg` (shared_ptr copy).
   *
   * Notes:
   * - Callback may run concurrently with `mapCallback`; mutex ensures atomic pointer swap and safe reads.
   * - Stale costmaps are acceptable until the next message; no explicit timeout logic.
   */
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0) return;
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    current_costmap_ = msg;
  }

  /**
   * @brief On each new SLAM map, detect frontiers, cluster, filter by costmap, publish poses and RViz markers.
   *
   * @param msg Latest `nav_msgs/OccupancyGrid` from the map topic (SLAM output).
   * @return void (publishes topics as side effects).
   *
   * Pipeline role:
   * - Main perception pipeline tick: this is where raw map becomes exploration goals for downstream nodes.
   * - Bridges mapping → (this node) → goal assignment / Nav2 interface via `PoseArray`.
   *
   * Implementation summary:
   * 1. Validate non-empty map; optionally log at INFO vs DEBUG based on `verbose_frontier_logging`.
   * 2. Scan all cells; collect `frontier_cells` where `isFrontierCell` is true.
   * 3. `clusterByAdjacency`; drop clusters smaller than `min_cluster_size`.
   * 4. For each surviving cluster, `clusterCentroid` → candidate (world) points.
   * 5. If `use_costmap_filter`, compare `getCostAtWorldWithIndices` to `cost_threshold`; partition into
   *    `safe_centroids` and `rejected_centroids` (cost -1 accepts regardless when filter on).
   * 6. Publish `PoseArray` of identity-orientation poses at safe centroids.
   * 7. Build `MarkerArray`: green spheres for safe, optional red for rejected, optional orange for raw cells.
   *
   * Notes:
   * - Map and costmap must overlap in frame/resolution expectations for filtering to be meaningful.
   * - Pose orientation is fixed (w=1); navigators typically replan heading at the goal.
   * - High log volume when verbose is on; default keeps detailed steps at DEBUG level.
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0) {
      RCLCPP_WARN(get_logger(), "Received empty map");
      return;
    }

    const bool verbose = get_parameter("verbose_frontier_logging").as_bool();
#define FRONTIER_LOG(...) do { \
  if (verbose) RCLCPP_INFO(get_logger(), __VA_ARGS__); \
  else RCLCPP_DEBUG(get_logger(), __VA_ARGS__); \
} while (0)

    const int w = static_cast<int>(msg->info.width);
    const int h = static_cast<int>(msg->info.height);
    std::vector<Cell> frontier_cells;
    for (int my = 0; my < h; my++) {
      for (int mx = 0; mx < w; mx++) {
        if (isFrontierCell(*msg, mx, my)) {
          frontier_cells.emplace_back(mx, my);
        }
      }
    }

    FRONTIER_LOG("[frontier] 1. raw frontier cells: %zu", frontier_cells.size());

    int min_size = get_parameter("min_cluster_size").as_int();
    std::vector<Cluster> clusters = clusterByAdjacency(frontier_cells, w, h);

    FRONTIER_LOG("[frontier] 2. total clusters: %zu (min_cluster_size=%d)", clusters.size(), min_size);

    std::vector<std::pair<double, double>> all_centroids;
    for (size_t ci = 0; ci < clusters.size(); ci++) {
      const Cluster & cl = clusters[ci];
      FRONTIER_LOG("[frontier] 3. cluster[%zu] size=%zu", ci, cl.size());
      if (static_cast<int>(cl.size()) < min_size) {
        FRONTIER_LOG("[frontier]    skipped (below min_cluster_size)");
        continue;
      }
      double cx, cy;
      clusterCentroid(*msg, cl, cx, cy);
      all_centroids.emplace_back(cx, cy);
      FRONTIER_LOG("[frontier] 4. cluster[%zu] centroid (representative): (%.3f, %.3f)", ci, cx, cy);
    }

    int cost_threshold = get_parameter("cost_threshold").as_int();
    bool use_filter = get_parameter("use_costmap_filter").as_bool();
    const int edge_margin = get_parameter("costmap_edge_margin").as_int();
    int cm_w = 0, cm_h = 0;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (current_costmap_) {
        cm_w = static_cast<int>(current_costmap_->info.width);
        cm_h = static_cast<int>(current_costmap_->info.height);
      }
    }
    std::vector<std::pair<double, double>> safe_centroids;
    std::vector<std::pair<double, double>> rejected_centroids;
    for (size_t i = 0; i < all_centroids.size(); i++) {
      const auto & c = all_centroids[i];
      int cmx = 0, cmy = 0;
      int cost = getCostAtWorldWithIndices(c.first, c.second, cmx, cmy);
      FRONTIER_LOG("[frontier] 6. candidate[%zu] world=(%.3f, %.3f) costmap_idx=(%d, %d) cost=%d",
        i, c.first, c.second, cmx, cmy, cost);
      if (use_filter && cm_w > 0 && cm_h > 0 && edge_margin > 0) {
        if (cmx < edge_margin || cmx >= cm_w - edge_margin ||
            cmy < edge_margin || cmy >= cm_h - edge_margin) {
          rejected_centroids.push_back(c);
          FRONTIER_LOG("[frontier] 8. candidate[%zu] REJECTED (too close to costmap edge, idx=(%d,%d) margin=%d)",
            i, cmx, cmy, edge_margin);
          continue;
        }
      }
      if (!use_filter || cost < 0) {
        safe_centroids.push_back(c);
        FRONTIER_LOG("[frontier] 8. candidate[%zu] ACCEPTED (no filter or cost<0)", i);
        continue;
      }
      if (cost <= cost_threshold) {
        safe_centroids.push_back(c); 
        FRONTIER_LOG("[frontier] 8. candidate[%zu] ACCEPTED (cost %d <= threshold %d)", i, cost, cost_threshold);
      } else {
        rejected_centroids.push_back(c);
        FRONTIER_LOG("[frontier] 8. candidate[%zu] REJECTED (cost %d > threshold %d)", i, cost, cost_threshold);
      }
    }

    FRONTIER_LOG("[frontier] 9. final safe published frontiers: %zu", safe_centroids.size());
#undef FRONTIER_LOG

    std::string frame_id = get_parameter("frame_id").as_string();
    if (frame_id.empty()) frame_id = msg->header.frame_id;

    RCLCPP_INFO(get_logger(),
      "[frontier] raw=%zu clusters=%zu centroids=%zu safe=%zu rejected=%zu",
      frontier_cells.size(), clusters.size(), all_centroids.size(), safe_centroids.size(), rejected_centroids.size());

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = msg->header.stamp;
    pose_array.header.frame_id = frame_id;
    pose_array.poses.resize(safe_centroids.size());
    for (size_t i = 0; i < safe_centroids.size(); i++) {
      pose_array.poses[i].position.x = safe_centroids[i].first;
      pose_array.poses[i].position.y = safe_centroids[i].second;
      pose_array.poses[i].position.z = 0.0;
      pose_array.poses[i].orientation.w = 1.0;
      pose_array.poses[i].orientation.x = 0.0;
      pose_array.poses[i].orientation.y = 0.0;
      pose_array.poses[i].orientation.z = 0.0;
    }
    frontiers_pub_->publish(pose_array);

    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker centroid_marker;
    centroid_marker.header.stamp = msg->header.stamp;
    centroid_marker.header.frame_id = frame_id;
    centroid_marker.ns = "frontier_centroids_safe";
    centroid_marker.id = 0;
    centroid_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    centroid_marker.action = visualization_msgs::msg::Marker::ADD;
    double scale = msg->info.resolution * 2.0;
    centroid_marker.scale.x = scale;
    centroid_marker.scale.y = scale;
    centroid_marker.scale.z = scale;
    centroid_marker.color.r = 0.0f;
    centroid_marker.color.g = 1.0f;
    centroid_marker.color.b = 0.0f;
    centroid_marker.color.a = 0.9f;
    centroid_marker.points.resize(safe_centroids.size());
    for (size_t i = 0; i < safe_centroids.size(); i++) {
      centroid_marker.points[i].x = safe_centroids[i].first;
      centroid_marker.points[i].y = safe_centroids[i].second;
      centroid_marker.points[i].z = 0.0;
    }
    ma.markers.push_back(centroid_marker);

    if (get_parameter("publish_rejected_markers").as_bool() && !rejected_centroids.empty()) {
      visualization_msgs::msg::Marker rej_marker;
      rej_marker.header.stamp = msg->header.stamp;
      rej_marker.header.frame_id = frame_id;
      rej_marker.ns = "frontier_centroids_rejected";
      rej_marker.id = 1;
      rej_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      rej_marker.action = visualization_msgs::msg::Marker::ADD;
      rej_marker.scale.x = scale;
      rej_marker.scale.y = scale;
      rej_marker.scale.z = scale;
      rej_marker.color.r = 1.0f;
      rej_marker.color.g = 0.0f;
      rej_marker.color.b = 0.0f;
      rej_marker.color.a = 0.8f;
      rej_marker.points.resize(rejected_centroids.size());
      for (size_t i = 0; i < rejected_centroids.size(); i++) {
        rej_marker.points[i].x = rejected_centroids[i].first;
        rej_marker.points[i].y = rejected_centroids[i].second;
        rej_marker.points[i].z = 0.0;
      }
      ma.markers.push_back(rej_marker);
    }

    if (get_parameter("publish_raw_markers").as_bool()) {
      visualization_msgs::msg::Marker raw_marker;
      raw_marker.header.stamp = msg->header.stamp;
      raw_marker.header.frame_id = frame_id;
      raw_marker.ns = "frontier_cells";
      raw_marker.id = 2;
      raw_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      raw_marker.action = visualization_msgs::msg::Marker::ADD;
      double raw_scale = msg->info.resolution * 1.2;
      raw_marker.scale.x = raw_scale;
      raw_marker.scale.y = raw_scale;
      raw_marker.scale.z = raw_scale;
      raw_marker.color.r = 1.0f;
      raw_marker.color.g = 0.5f;
      raw_marker.color.b = 0.0f;
      raw_marker.color.a = 0.4f;
      raw_marker.points.resize(frontier_cells.size());
      for (size_t i = 0; i < frontier_cells.size(); i++) {
        double wx, wy;
        gridToWorld(*msg, frontier_cells[i].first, frontier_cells[i].second, wx, wy);
        raw_marker.points[i].x = wx;
        raw_marker.points[i].y = wy;
        raw_marker.points[i].z = 0.0;
      }
      ma.markers.push_back(raw_marker);
    }
    markers_pub_->publish(ma);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontiers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
  mutable std::mutex costmap_mutex_;
};

}  // namespace tb3_frontier_exploration

/**
 * @brief Program entry: initialize ROS 2 and spin one `FrontierDetectionNode` until shutdown.
 *
 * @param argc Standard C argument count.
 * @param argv Standard C argument vector (ROS 2 may remap/consume arguments).
 * @return 0 after clean shutdown.
 *
 * Pipeline role:
 * - Starts the frontier detection process as a standalone executable; without `main`, the node would not run.
 *
 * Implementation summary:
 * 1. `rclcpp::init`.
 * 2. `spin` on a new `FrontierDetectionNode` shared_ptr.
 * 3. `rclcpp::shutdown` and return 0.
 *
 * Notes:
 * - Single-threaded spinner; callbacks for map and costmap run on the same thread in sequence.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tb3_frontier_exploration::FrontierDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
