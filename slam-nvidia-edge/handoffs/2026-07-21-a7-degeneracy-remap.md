# Handoff — 2026-07-21 — a7-degeneracy-remap

**Session model tier:** fable (Opus), parallel with an A8 sonnet delegate (worktree)
**Activities touched:** A7 (done)

## Done

- **A7** (`lio_node/degeneracy_detector.hpp` + `lio_node.cpp`): Zhang & Singh
  solution remapping. `observableProjection(H)` builds P_obs = Σ v_i v_iᵀ over
  eigenvectors whose eigenvalue clears `max_ev / condition_threshold`; `remap()`
  projects the 6-DoF ICP increment ([rot, trans] basis, matching the A5 Hessian)
  onto that subspace. `analyze()` now also reports `num_degenerate_directions`.
  Wired into `lio_node.cpp::onScan`: when degenerate, the increment is
  decomposed to a 6-vector (logSO3 of the rotation increment + translation),
  remapped, and recomposed — so the IMU prediction is retained along
  unobservable directions instead of drifting.
- New `tests/test_degeneracy_remap.cpp` (7th suite): well-conditioned → identity
  projection; two sub-cutoff eigenvalues in a rotated basis → those components
  zeroed and observable ones preserved; physical single-plane Hessian → in-plane
  translation removed, normal + rotation kept; idempotent + symmetric; zero
  Hessian → full projection-out.

## Decisions

- **Observability cutoff reuses the existing condition-number threshold**
  (`max_ev / threshold_`) rather than a new absolute-eigenvalue param — a
  direction is unobservable exactly when it would push the condition number
  past threshold. Keeps the single-arg `DegeneracyDetector` ctor and its
  existing test green; no contract change.
- Remapping is applied **post-solve** to the increment (not inside the GN
  solve). This is the standard skeleton-grade approximation and is honest about
  it in the code comment; a fuller version would remap within the ICP normal
  equations (revisit if A14's KD-tree changes the Hessian conditioning).

## Next

- A8 (error-state EKF fusion) is being implemented by a sonnet delegate in a
  worktree with a full inline design; on completion, fable-review its diff,
  merge its files into the main tree, wire backend selection in
  fusion_node.cpp, run the full suite, commit. Then A9 (GPS/wheel) unblocks.
- A4 (fixed-lag smoother, fable design) remains the other big open item.

## Verification

- `ctest --test-dir tests/build --output-on-failure`: "100% tests passed,
  0 tests failed out of 7".
- `g++ -fsyntax-only` clean on `degeneracy_detector.hpp`.
