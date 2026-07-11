# Handoff — 2026-07-11 — m1-agent-protocol

**Session model tier:** fable
**Activities touched:** protocol/docs (M1); M0 stack merged earlier today (PR #1)

## Done

- M0 (same session, merged as PR #1, merge commit `820ff62`): full skeleton
  stack — 10 packages, 4 passing core test suites, Docker/bringup/docs.
- M1 (this handoff's PR): agent protocol — `CLAUDE.md`, `STATE.md`,
  `docs/agent_protocol.md`, `docs/code_graph.{json,md}`, three skills under
  `.claude/skills/`, this handoffs/ directory.

## Decisions

- **slam_core exists** (addition to the originally requested layout) so shared
  math lives once, ROS-free, unit-testable off-robot. Do not fold it back into
  node packages.
- **Contracts are frozen at sonnet tier**: `slam_interfaces` msgs and the
  module output structs (`TrackResult`, `IcpResult`, `EstimatorOutput`,
  `FrontendFeatures`, `FusedState`) change only with fable sign-off. This is
  what makes cheap-model delegation safe.
- **GPS/wheel fusion weights are pinned to zero** until frame alignment (A9)
  exists — do not "fix" them to non-zero as a quick win.
- **Custom one-screen test harness** (`tests/test_framework.hpp`) instead of
  gtest, so the acceptance tests run anywhere with just Eigen. Keep it.
- Skills under `slam-nvidia-edge/.claude/skills/` are loaded by explicit read
  (CLAUDE.md step 4), not auto-discovery, since sessions root at the repo top.

## Next

- A12 (CI workflow; sonnet design → haiku maintain) — do this first: it turns
  the Docker/colcon gate from "please run on a Jazzy host" into an automatic
  PR check, which every later delegation relies on.
- Then the independent sonnet lanes in parallel: A3 (tracker), A6 (deskew),
  A10 (TrtEngine).
- A1 (preintegration covariance; fable) unblocks the A4/A5/A8 critical path.

## Verification

- `ctest --test-dir tests/build`: 100% tests passed, 0 tests failed out of 4
  (re-run this session after branch reset).
- `python3 -m json.tool docs/code_graph.json`: parses clean.
- `colcon build` still unverified on a ROS 2 host — tracked in STATE.md.
