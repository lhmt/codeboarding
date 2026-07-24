# STATE — slam-nvidia-edge

> Living session-state file. Every session reads this first and updates it
> before pushing (see `CLAUDE.md` and `.claude/skills/slam-handoff/`).

## Current milestone

**M1 — agent protocol + delegation infrastructure** (in progress)
M0 (full skeleton stack, merged in PR #1, 2026-07-11): all 10 packages compile
targets defined, 4 core test suites passing, Docker/bringup/docs complete.

## Component status

| Component | Maturity | Notes |
|---|---|---|
| slam_interfaces | contract-frozen | Msg changes require fable + version note here |
| slam_core | functional-math | Preintegration covariance + bias Jacobians done (A1) |
| vio_node | skeleton | Placeholder tracker + dead-reckoning window (A3, A4) |
| lio_node | functional-core | Deskew (A6), real GN ICP (A5), Zhang-Singh degeneracy remap (A7) all done |
| fusion_node | skeleton | Weighted avg default; error-state EKF backend in progress (A8, delegated); GPS/wheel (A9) |
| tensor_frontend_node | skeleton | Null engine; TrtEngine stub not deserializing (A10) |
| health_monitor_node | functional | Rules implemented + tested |
| map_node | relay-only | Persistent mapping TODO (A11) |
| recorder_node | functional | Local artifacts only; uploader out of scope |
| slam_bringup / configs / docker | functional | CI not wired yet (A12) |
| tests/ | passing 4/4 | ROS-free; last verified 2026-07-11 (g++13, Eigen 3.4) |

## Work queue (tier-tagged; full definitions in docs/code_graph.json)

| # | Activity | Tier | Blocked by |
|---|---|---|---|
| A12 | CI workflow — **landed in PR #2**; blocked by ACCOUNT-LEVEL GitHub Actions outage (14 attempts since 07-12: no runner ever assigned; fix is in GitHub billing/Actions settings) | sonnet (delegated ✓) | GitHub account fix |
| A13 | Wire PointCloud2 per-point time fields into the new deskew overload | sonnet | driver with time field |
| A2 | Swap slam_core lie_group placeholder for Sophus | sonnet | A12 (CI gate first) |
| A3 | Real feature tracker (KLT or frontend features) in vio_node | sonnet | — |
| A10 | TrtEngine: deserialize .engine, enqueueV3, decode SuperPoint heads | sonnet | — |
| A8 | Error-state EKF fusion backend | fable design → sonnet impl (in progress) | — (A1 done) |
| A4 | Sliding-window backend (Ceres/GTSAM fixed-lag) | fable design → sonnet impl | — (A1 done) |
| A9 | GPS ENU anchoring + wheel-odom frame alignment | sonnet | A8 |
| A11 | Persistent map / nvblox integration in map_node | sonnet | — (A5 done) |
| A14 | KD-tree/ikd-tree correspondence search in ICP (perf; brute-force O(N·M) now) | sonnet | — |
| A3 | Real feature tracker in vio_node (needs ROS+OpenCV; not in ROS-free harness) | sonnet | — |
| A10 | TrtEngine deserialize/enqueue/decode (needs TensorRT) | sonnet | — |

Done: A1 (preint covariance+Jacobians, fable), A6 (deskew, sonnet), A5 (GN point-to-plane ICP, sonnet+fable review), A7 (Zhang-Singh degeneracy remap, fable) — all 2026-07-21.

## Invariants (do not break; escalate if a task appears to require it)

1. No cloud/network dependency in any runtime node (docs/sagemaker_boundary.md).
2. `slam_core` stays header-only, ROS-free, Eigen-only.
3. Top-level `tests/` must build and pass with plain CMake + Eigen (no ROS).
4. Published topic names/types in docs/dataflow.md are the external contract.
5. Deterministic estimation path: no RNG, single-threaded executors.

## Last verification

- 2026-07-21: `ctest` 7/7 pass (adds degeneracy_remap: observable-subspace
  projection zeros unobservable eigen-directions in a rotated basis and on a
  physical single-plane Hessian, idempotent+symmetric; point_to_plane_icp
  recovery ~1e-6 m / 1e-12 rad); headers pass `g++ -fsyntax-only`
  (C++20, `-Wall -Wextra -Wpedantic`).
- `colcon build` / Docker image build: **not yet run on a ROS 2 Jazzy host** —
  blocked by the account-level GitHub Actions outage (see A12); update on
  first green CI run or on-target build.
