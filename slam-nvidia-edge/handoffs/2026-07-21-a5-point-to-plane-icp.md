# Handoff — 2026-07-21 — a5-point-to-plane-icp

**Session model tier:** sonnet delegate (impl) + fable orchestrator (review)
**Activities touched:** A5 (done), A14 (filed)

## Done

- **A5** (`lio_node/point_to_plane_icp.hpp`): real Gauss-Newton point-to-plane
  ICP replacing the identity placeholder. Brute-force k=6 NN correspondences
  (EXTENSION POINT for KD-tree, filed as A14), plane fit via smallest
  eigenvector of the neighbor scatter, Jacobian order [phi, t], 1e-9 Levenberg
  damping, ldlt solve, left-multiplicative update. `IcpResult` contract and
  `align()` signature unchanged; new defaulted `epsilon` ctor param keeps
  lio_node.cpp compiling untouched. Real J^T J now feeds the degeneracy
  detector.
- New `tests/test_point_to_plane_icp.cpp`: perturbation recovery on a
  972-point three-plane fixture (~1e-6 m / ~1e-12 rad final error, converged
  in 2 iterations, fitness 1.0), PSD Hessian, single-plane degeneracy flagged
  (condition ~9e12), empty-input behavior.

## Decisions

- **Fable review fix applied:** rotation-block Jacobian uses
  `hat(R*p + t)` (fully transformed point, left-perturbation derivation), not
  the delegate's original `hat(R*p)` — equivalent at t=0 but wrong for larger
  translations, and the degeneracy detector consumes this Hessian's spectrum.
- Test fixture excludes a 0.3 m margin around plane-intersection axes: with
  brute-force k-NN, neighbors legitimately mix across planes there and bias
  the fit. Fixture-geometry fix chosen over a planarity-ratio rejection check
  (spec'd out of A5 to keep the solver simple); revisit if A14's KD-tree
  changes neighbor statistics.
- Delegation observation for the protocol: the delegate self-diagnosed the
  fixture bias correctly and did not weaken tolerances — the "acceptance
  criteria are a test" pattern is working.

## Next

- A7 (degeneracy solution remapping, fable) now unblocked — natural next since
  the ICP Hessian is real. A11 (persistent map) also unblocked. A4/A8 design
  docs (fable) remain open; A3/A10/A14 are open sonnet lanes.

## Verification

- `ctest --test-dir tests/build --output-on-failure` after the review fix:
  "100% tests passed, 0 tests failed out of 6".
- Strict syntax gate clean on the changed header.
