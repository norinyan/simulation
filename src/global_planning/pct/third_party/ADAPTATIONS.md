The upstream revision is pinned in UPSTREAM. Original sources retain GPL-2.0-or-later; OSQP retains its own license.

Changes to upstream C++ files:
- OSQP include spelling matches an in-tree build rather than an installed osqp/ prefix.
- Astar exposes optional stop and transition validation callbacks, with no change in heuristic/edge weights/queue policy. OfflineElePlanner forwards these callbacks.
- OfflineElePlanner exposes ConfigureOptimizer to forward GPMP sample-interval, iteration-limit and jerk-sigma parameters. The adapter exposes the upstream safe-cost target instead of hard-coding 15.
- OfflineElePlanner::OptimizePath optimizes the last search result; Plan still offers the upstream search-and-optimize entry. The adapter validates the first search, then calls OptimizePath, avoiding a duplicate search.
- DenseElevationMap bilinear interpolation uses integer-indexed cell coordinates consistently: floor(coord), not floor(coord-.5) with unshifted interpolation weights. The previous code extrapolated with weights above 1 and jumped at half-cell boundaries for nonlinear cost profiles. Queries are clamped to the map. A regression test checks continuity, cell values, gradients and boundaries.
- GPMP initializes subsampled headings and speeds from centered position differences and its own dt, rather than reusing quantized single-grid-edge headings with a unit speed. This changes the initial guess, not the objective.
- GPMP jerk sigma is configurable (upstream 0.1); the current task profile uses 0.03. Objective/factor formulas and the height-smoothing objective remain unchanged.

Original GPU scripts are in planning/src/map_processing/pct. Tomogram accepts a simplify_layers flag; this integration disables layer removal. The wrapper handles geometry sampling, unknown cells, serialization, footprint inflation and world coordinates. It does not replace the original kernels with another mapper.

The build uses system GTSAM and matching default Eigen alignment, with no -march=native. Original GPMP and height smoother are retained and enabled by default in the ROS 2 launch entry after task-specific tuning; failed output validation is an explicit failure.

New workspace integration: the runtime-only core is retained without ROS 1/Python wrappers or upstream tests. The ROS adapter preserves the ground search route and labels edges using actual World tread/landing boxes; no company execution route replacement is used. The map generator and path height-jump check use a 1e-6 m float32 numeric tolerance while the cache continues to record the physical 0.16 m step bound. Robot nominal standing envelope is 0.605 x 0.78 m with a 0.05 m margin and 0.62 m ground-to-top clearance, derived from the current URDF/control configuration; this is not a complete foot swing/turn swept-body guarantee.
The adapter consistently applies the configured cost threshold to search, edge/path checks and optimized-reference checks. The existing inflation contour formula uses (1 - cost_threshold / cost_barrier), equal to the original 0.6 at 20/50; both values participate in offline cache metadata/fingerprinting.
