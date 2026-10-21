# Frontier Detection — Debug Logging Guide

## How to see detailed logs

- **Option 1:** Set the node log level to DEBUG when launching:  
  `--log-level frontier_detection_node:=DEBUG`
- **Option 2:** Set `verbose_frontier_logging` to `true` in `params.yaml`; then the step-by-step logs are printed at **INFO** level without changing the global log level.

## Log format per detection cycle (steps 1–9)

| Step | Meaning | Example |
|------|--------|---------|
| **1** | **Raw frontier cell count** for this cycle (free cells with at least one unknown in 8-neighborhood) | `[frontier] 1. raw frontier cells: 142` |
| **2** | **Number of clusters** after adjacency clustering of raw cells, and `min_cluster_size` | `[frontier] 2. total clusters: 12 (min_cluster_size=5)` |
| **3** | **Size of each cluster**; if below `min_cluster_size` the cluster is skipped | `[frontier] 3. cluster[0] size=8`, `skipped (below min_cluster_size)` |
| **4** | **Geometric centroid** of each kept cluster (used as representative point), world (x, y) | `[frontier] 4. cluster[0] centroid (representative): (2.100, -0.500)` |
| **6** | For each **candidate centroid**: world coords, **costmap indices (mx, my)**, and **sampled cost value** | `[frontier] 6. candidate[0] world=(2.100, -0.500) costmap_idx=(42, 31) cost=0` |
| **8** | Whether the candidate is **ACCEPTED** or **REJECTED** and why (no filter, cost&lt;0, or cost vs threshold) | `[frontier] 8. candidate[0] ACCEPTED (cost 0 <= threshold 128)` |
| **9** | **Final number of safe published frontiers** (poses in `/frontiers` PoseArray) | `[frontier] 9. final safe published frontiers: 3` |

Each cycle also prints one **INFO** summary line, e.g.:

```text
[frontier] raw=142 clusters=12 centroids=5 safe=3 rejected=2
```

Meaning: 142 raw cells → 12 clusters → 5 centroids (passing min_cluster_size) → after cost filtering 3 safe, 2 rejected.

## How to interpret logs when only one frontier is published

Work through in order:

1. **Step 1**  
   - If **raw frontier cells** is consistently low (e.g. &lt; 10), there are few frontier cells in the map—e.g. map not yet explored or very open space. Check `/map` and raw markers in RViz.

2. **Steps 2 and 3**  
   - If **total clusters** is high but many clusters are **skipped** because `size < min_cluster_size`, the number of **centroids** will be much smaller than the number of clusters.  
   - If `min_cluster_size` is too large, only a few (or one) large clusters remain, so you get only one centroid.  
   - **Suggestion:** Look at each cluster size in step 3; if many clusters are just below `min_cluster_size`, try **lowering `min_cluster_size`** or check whether resolution/inflation is breaking frontiers into many small clusters.

3. **Steps 6 and 8**  
   - If you have multiple centroids but step 8 shows most as **REJECTED (cost X > threshold 128)**, the costmap has high cost at those positions (obstacle or inflation).  
   - **Suggestion:** Check each candidate’s `costmap_idx` and `cost`; to allow more frontiers, **increase `cost_threshold`** or review costmap inflation/layers.  
   - If `cost=-1` or the costmap is not ready, the current logic treats the candidate as ACCEPTED (no filter or invalid cost counts as pass).

4. **Step 9**  
   - **Final safe published frontiers** is the number of poses actually published to `/frontiers`.  
   - If this is always 1, then from 1–3 above: either only one centroid is produced (clustering / `min_cluster_size`), or only one of several centroids passes the cost filter.

**Summary:** Use steps 1→2→3 to see how many centroids you get, then steps 6/8 to see which are filtered by cost, and step 9 plus the INFO summary for the final count. Adjust `min_cluster_size` or `cost_threshold` accordingly to fix “only one frontier” behavior.
