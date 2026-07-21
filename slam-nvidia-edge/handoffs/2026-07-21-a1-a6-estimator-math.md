# Handoff — 2026-07-21 — a1-a6-estimator-math

**Session model tier:** fable (A1) + sonnet delegate (A6), parallel lanes
**Activities touched:** A1 (done), A6 (done), A13 (filed)

## Done

- **A1** (`slam_core/imu_preintegration.hpp`): 9x9 measurement-noise covariance
  propagation (state order [dphi, dv, dp]; discrete sample variance sigma^2/dt)
  and the five first-order bias-update Jacobians (Forster RSS'15 eq. 59-64),
  plus `biasCorrectedDelta()`. Bias random walk deliberately excluded from this
  covariance — it belongs to the separate bias-evolution factor.
  `ImuPreintegrator` gained an optional `ImuNoiseParams` ctor arg; existing
  callers compile unchanged (default = zero densities = zero covariance).
- **A6** (sonnet delegate, `lio_node/scan_deskew.hpp`): time-fraction overload
  rotating each point into the scan-end frame via `expSO3(-(1-tau) * phi)`;
  clamped tau; size-mismatch passthrough; original overload untouched.
- Tests: `test_imu_preintegration` extended (numeric bias-Jacobian check
  against full re-integration, tolerance 1e-6, with a non-triviality guard;
  covariance symmetric/PSD/growing/zero-without-noise); new
  `test_scan_deskew` (5 checks). Suite is now 5 binaries.

## Blocked

- A12 (CI green) still blocked by the account-level GitHub Actions outage
  (14 probe attempts since 07-12, no runner ever assigned). User must fix
  GitHub billing/Actions settings; watch trigger re-arms ~12h.

## Decisions

- Covariance state order is [dphi, dv, dp] (9x9). Any solver consuming
  `covariance()` must use this order.
- Deskew is rotation-only; translation deskew is an explicit EXTENSION POINT.
- lio_node.cpp still calls the passthrough deskew overload — PointCloud2
  time-field parsing filed as **A13** rather than bundled into A6.

## Next

- A4/A5/A8 are now unblocked (A1 done). Recommended order: A5 (real ICP,
  sonnet + fable review) since A7 and A11 chain behind it; A4/A8 design docs
  (fable) can proceed in parallel.
- Independent sonnet lanes still open: A3 (tracker), A10 (TrtEngine).

## Verification

- `ctest --test-dir tests/build --output-on-failure`:
  "100% tests passed, 0 tests failed out of 5" (verified independently by
  both the orchestrator and the A6 delegate on clean build dirs).
- `g++ -fsyntax-only` clean on changed headers (C++20, -Wall -Wextra -Wpedantic).
- Note: -Wmaybe-uninitialized warnings at -O2 from inside Eigen 3.4 headers
  (SelfAdjointEigenSolver path) are a known GCC-13 false positive, not ours.
