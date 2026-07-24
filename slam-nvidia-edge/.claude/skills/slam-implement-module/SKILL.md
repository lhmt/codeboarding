---
name: slam-implement-module
description: Procedure for implementing or replacing an extension-point module (tracker, ICP, deskew, fusion backend, TensorRT engine) in slam-nvidia-edge.
---

# Implement an extension-point module

## Before writing code

1. Find your activity in `docs/code_graph.json` — it lists files, tier,
   blockers, and an `acceptance` clause. If your task has no activity entry,
   check STATE.md's queue; if it's not there either, escalate (scope decision).
2. Read the target header top-to-bottom. Every placeholder marks its insertion
   point with `EXTENSION POINT:` or `TODO(...)` and states which parts of the
   struct/return contract are frozen.
3. Confirm your model tier ≥ the file's tier (`sub_tiers` in code_graph.json).

## Rules

- **The output contract is frozen.** `TrackResult`, `IcpResult`,
  `EstimatorOutput`, `FrontendFeatures`, `FusedState` field sets do not change
  without fable sign-off. Add capability by filling fields that are currently
  placeholder-valued, not by reshaping structs.
- Estimator math goes in `slam_core` (ROS-free); node packages hold plumbing
  and sensor-format handling only.
- Tests first when the activity has an `acceptance` clause: encode it as a
  test in `tests/` (ROS-free) before implementing.
- New third-party deps: add to `package.xml` + `CMakeLists.txt` +
  `docker/Dockerfile.jetson` in the same commit, or the Docker gate breaks.
- Keep determinism: no RNG, no wall-clock in the estimation path, no threads
  beyond the node executor.

## Escalation triggers (stop, report, hand up)

Verbatim list in `CLAUDE.md`. The short version: contract changes, slam_core
math, topic/QoS changes, network deps, twice-failed fix, out-of-scope test
failures.

## Done means

`slam-build-test` gates green for your tier + acceptance test passing +
STATE.md and a handoff updated via `slam-handoff`.
