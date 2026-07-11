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
| slam_core | skeleton+ | Math correct for accumulation; covariance/Jacobians TODO (A1) |
| vio_node | skeleton | Placeholder tracker + dead-reckoning window (A3, A4) |
| lio_node | skeleton | ICP returns identity; deskew passthrough (A5, A6, A7) |
| fusion_node | skeleton | Weighted average only; GPS/wheel at zero weight (A8, A9) |
| tensor_frontend_node | skeleton | Null engine; TrtEngine stub not deserializing (A10) |
| health_monitor_node | functional | Rules implemented + tested |
| map_node | relay-only | Persistent mapping TODO (A11) |
| recorder_node | functional | Local artifacts only; uploader out of scope |
| slam_bringup / configs / docker | functional | CI not wired yet (A12) |
| tests/ | passing 4/4 | ROS-free; last verified 2026-07-11 (g++13, Eigen 3.4) |

## Work queue (tier-tagged; full definitions in docs/code_graph.json)

| # | Activity | Tier | Blocked by |
|---|---|---|---|
| A12 | CI: GitHub Actions job running scripts/build.sh in ros:jazzy container | haiku→sonnet | — |
| A2 | Swap slam_core lie_group placeholder for Sophus | sonnet | A12 (CI gate first) |
| A1 | Preintegration covariance + bias Jacobians (Forster eq. 62-64) | fable | — |
| A3 | Real feature tracker (KLT or frontend features) in vio_node | sonnet | — |
| A10 | TrtEngine: deserialize .engine, enqueueV3, decode SuperPoint heads | sonnet | — |
| A6 | Deskew with per-point time fields | sonnet | — |
| A5 | Real point-to-plane ICP (small_gicp or hand-rolled GN) | sonnet+fable review | A1 |
| A4 | Sliding-window backend (Ceres/GTSAM fixed-lag) | fable design → sonnet impl | A1 |
| A7 | Degeneracy solution remapping (Zhang & Singh) | fable | A5 |
| A8 | Error-state EKF fusion backend | fable design → sonnet impl | A1 |
| A9 | GPS ENU anchoring + wheel-odom frame alignment | sonnet | A8 |
| A11 | Persistent map / nvblox integration in map_node | sonnet | A5 |

## Invariants (do not break; escalate if a task appears to require it)

1. No cloud/network dependency in any runtime node (docs/sagemaker_boundary.md).
2. `slam_core` stays header-only, ROS-free, Eigen-only.
3. Top-level `tests/` must build and pass with plain CMake + Eigen (no ROS).
4. Published topic names/types in docs/dataflow.md are the external contract.
5. Deterministic estimation path: no RNG, single-threaded executors.

## Last verification

- 2026-07-11: `ctest` 4/4 pass; all ROS-free headers pass `g++ -fsyntax-only`
  (C++20, `-Wall -Wextra -Wpedantic`); launch files black-clean.
- `colcon build` / Docker image build: **not yet run on a ROS 2 Jazzy host** —
  first CI run (A12) or on-target build should confirm and update this line.
