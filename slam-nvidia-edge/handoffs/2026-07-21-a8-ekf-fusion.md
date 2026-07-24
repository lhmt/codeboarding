# Handoff — 2026-07-21 — a8-ekf-fusion

**Session model tier:** sonnet delegate (worktree impl) + fable orchestrator (review + merge + wiring)
**Activities touched:** A8 (done)

## Done

- **A8** (`fusion_node/ekf_backend.hpp`, new): `ErrorStateEkfBackend`
  implementing the frozen `FusionBackend` interface. Persistent nominal
  `[p, v, q]` + 9x9 error-state covariance `P` (order [dp, dv, dtheta]),
  uninformed `100*I` prior, hard-init to the highest-weight source on first
  non-empty `fuse()`. Per call: process-noise inflation `P += Q` (no dt in the
  interface — EXTENSION POINT for a dt-driven constant-velocity predict), then
  sequential Kalman updates per alive source (position + velocity always,
  orientation when `has_orientation`), weight → inverse measurement variance.
- Wired into `fusion_node.cpp`: `backend: ekf` param selects it; `weighted`
  remains the default. Interface, WeightedAverageBackend, slam_core, and
  slam_interfaces all untouched.
- New `tests/test_ekf_fusion.cpp` (8th suite): empty→nullopt, single-source
  convergence, two-source covariance shrinkage, weighted disagreement
  (x≈8.0), orientation recovery, covariance symmetric+PSD.

## Decisions

- **Fable review fix (real bug):** the delegate's orientation residual was
  `Log(q_nominal^-1 · q_meas)` (body frame) while the injection is a **left**
  multiply `Exp(dx)·q_` (world frame) — inconsistent. The convergence test
  masked it because the fixed point has zero residual in either frame. Changed
  the residual to `Log(q_meas · q_nominal^-1)` (world frame) to match the
  left-perturbation error state. Suite still green.
- EKF holds state as member data because `FusionBackend::fuse()` is
  stateless-per-call; this keeps the interface frozen (no dt/predict args), at
  the cost of a constant (not dt-scaled) process model — acceptable for now,
  flagged in-code.
- Delegate ran in an isolated **worktree** (pinned at the M0 base commit) to
  avoid racing my parallel A7 edits on `tests/CMakeLists.txt`; only disjoint
  new files were copied into the main tree and the CMake block hand-merged.
- Root `/.claude/` (agent worktree scratch) added to `.gitignore`.

## Next

- A9 (GPS ENU anchoring + wheel-odom alignment, sonnet) now unblocked — lets
  the currently-zero-weight GPS/wheel sources contribute to the EKF.
- A4 (fixed-lag smoother, fable design) is the remaining large item.
- Consider an integration test exercising fusion_node with the EKF backend
  once a ROS test harness exists (current suite is ROS-free).

## Verification

- `ctest --test-dir tests/build --output-on-failure`: "100% tests passed,
  0 tests failed out of 8".
- `g++ -fsyntax-only` clean on `ekf_backend.hpp`.
