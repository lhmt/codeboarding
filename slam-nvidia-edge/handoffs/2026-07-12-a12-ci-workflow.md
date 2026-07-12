# Handoff — 2026-07-12 — a12-ci-workflow

**Session model tier:** fable orchestrator + sonnet delegate (first live use of the routing protocol)
**Activities touched:** A12

## Done

- `.github/workflows/slam-edge-ci.yml` (repo root — GitHub requirement), written
  by a sonnet-tier delegate from a fixed spec, fable-reviewed before commit.
  Two jobs, both path-filtered to `slam-nvidia-edge/**` + the workflow file:
  `core-tests` (ubuntu-24.04 + Eigen, ctest gate) and `colcon-build`
  (ros:jazzy container, full workspace build).

## In flight

- A12 closes on the first green run (triggers on PR #2 itself). If
  `colcon-build` surfaces Jazzy API issues in node code, that is new A-queue
  work — file it, don't patch it inside this activity.

## Decisions

- CI does **not** build `docker/Dockerfile.jetson` (Sophus source build makes
  it too slow for a PR gate); the ros:jazzy container job is the workspace
  gate, the Docker image remains a release/on-target artifact.
- Delegation protocol validated end-to-end: sonnet delegate received activity
  spec + verification gate + escalation triggers, hit no escalations, returned
  pasted verification (YAML parse OK; ctest 4/4).

## Next

- Watch PR #2 CI; when green: flip A12 to done in STATE.md, update
  "Last verification", unblock A2 (Sophus swap, sonnet).
- Independent sonnet lanes remain open: A3 (tracker), A6 (deskew), A10 (TrtEngine).

## Verification

- Delegate: `yaml.safe_load` on the workflow → OK; ctest re-run → "100% tests
  passed, 0 tests failed out of 4".
- Orchestrator review: path filters, least-privilege `permissions`,
  sh-compatible `.` sourcing with `shell: bash`, no existing workflows touched.
