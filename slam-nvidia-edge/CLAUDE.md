# slam-nvidia-edge — agent session protocol

Scope: work under `slam-nvidia-edge/`. The repo-root AGENTS.md governs the
CodeBoarding Python project; only its git/PR conventions apply here.

## Session start (every session, every model tier)

1. Read `STATE.md` — current milestone, component status, work queue, invariants.
2. Read the newest file in `handoffs/` — what the last session did and left open.
3. Scope the task against `docs/code_graph.json`: find the component(s) you will
   touch and check their `edit_tier`. If your tier is below the component's
   tier, stop and escalate (see routing below).
4. Load the matching skill from `.claude/skills/` before acting:
   - building/verifying → `slam-build-test`
   - implementing an extension point → `slam-implement-module`
   - ending the session → `slam-handoff`

## Session end (mandatory, before push)

Run the `slam-handoff` skill: update `STATE.md` (status table + work queue) and
append a handoff file under `handoffs/`. A session that changed code but left
STATE.md stale is an incomplete session.

## Build & verify

```bash
./scripts/build.sh                        # core tests always; colcon when available
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
docker build -f docker/Dockerfile.jetson .   # full ROS build gate (runs tests + colcon)
```

## Model routing (delegation policy)

Full policy + escalation triggers: `docs/agent_protocol.md`. Summary:

| Tier | Use for |
|---|---|
| **haiku** | Mechanical, fully-specified, low blast radius: config/doc edits, renames, param additions, running builds/tests and reporting, scaffolding from an existing pattern |
| **sonnet** | Standard engineering against a stable contract: implement extension points behind existing interfaces, unit tests, reproducible bug fixes, launch/bringup, TensorRT/CI plumbing |
| **fable** | Reasoning-heavy or contract-changing: estimator math, solver/backend design, degeneracy handling, `slam_interfaces` msg changes, safety-boundary decisions, ambiguity |

Hard escalation triggers (any tier must stop and hand up to fable):
- diff touches `slam_core/` math or `slam_interfaces/msg/`
- a published topic name, type, or QoS changes
- the change would add any network dependency to the runtime (cloud boundary)
- two failed attempts at the same fix, or tests fail outside your task's scope

## Coding rules (unchanged from M0)

- C++20, `-Wall -Wextra -Wpedantic`; every package has `CMakeLists.txt` + `package.xml`.
- Deterministic control flow; no RNG in the estimation path; no cloud calls in runtime.
- Don't overbuild; keep interfaces stable and mark real-algorithm insertion points.
- Estimator math goes in `slam_core` (header-only, ROS-free, Eigen only) so it
  stays unit-testable off-robot; keep the top-level `tests/` buildable without ROS.
- Launch `.py` files must be black-clean (line length 120 — repo pre-commit).
