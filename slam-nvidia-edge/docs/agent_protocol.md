# Agent protocol: state, handoff, skills, model routing

How multi-session, multi-model agent work on this codebase stays coherent and
cheap. `CLAUDE.md` is the operational summary; this is the full spec.

## 1. State protocol

`STATE.md` is the single source of session-to-session truth:

- **Current milestone** — one line, what phase the project is in.
- **Component status** — maturity per package; the first thing a new session
  scans to avoid re-deriving the codebase.
- **Work queue** — tier-tagged activity IDs (defined in `code_graph.json`),
  ordered by priority, with blockers.
- **Invariants** — rules that survive every refactor; a task that seems to
  require breaking one is automatically a fable-tier decision.
- **Last verification** — what was proven to work, when, and on what toolchain.

Rules:
- Read before any work; update before any push. Stale STATE.md = broken handoff.
- Keep it under ~100 lines. Detail belongs in handoffs or docs, not here.
- Only the session that did the work updates it (no speculative edits).

## 2. Handoff protocol

Each session that changes code appends `handoffs/YYYY-MM-DD-<slug>.md`
(template: `handoffs/TEMPLATE.md`) with exactly these sections:

- **Done** — merged/pushed work, with commit SHAs.
- **In flight** — started but unfinished, with the exact file/function to resume at.
- **Blocked** — what stopped, why, what would unblock it.
- **Decisions** — choices a future session must not silently reverse
  (e.g. "hemisphere-aligned quaternion blend chosen over SLERP chain: N sources").
- **Next** — recommended next activities, tier-tagged.
- **Verification** — commands run and their actual results (paste, don't summarize).

Handoffs are append-only history; STATE.md is current truth. When they
disagree, STATE.md wins and the discrepancy is worth a line in your handoff.

## 3. Skill loading

Skills live in `.claude/skills/<name>/SKILL.md` under `slam-nvidia-edge/`.
Sessions rooted at the repo root won't auto-discover them — that is why
`CLAUDE.md` step 4 says to open the matching SKILL.md explicitly at task start.
Load at most what the task needs:

| Task shape | Skill |
|---|---|
| Build, test, verify, report status | `slam-build-test` |
| Implement/replace an extension-point module | `slam-implement-module` |
| End of any code-changing session | `slam-handoff` (mandatory) |

Skills are procedures, not knowledge dumps: each fits in one screen and links
to code rather than duplicating it.

## 4. Model routing

Objective: Haiku/Sonnet do the volume; Fable is reserved for work where
reasoning depth changes the outcome.

### Tier definitions

**haiku — mechanical.** The task is fully specified by an existing pattern and
verification is a command, not a judgment call. Examples: add a config
parameter with default + YAML entry; doc updates; topic rename across files;
run `./scripts/build.sh` / ctest and report; scaffold a package by copying
`map_node`'s shape; CI YAML edits after the workflow design exists.

**sonnet — engineering against a stable contract.** The interface exists, the
tests define correctness, and the change stays inside one component. Examples:
implement `TrtEngine::infer` behind `InferenceEngine`; real KLT tracker behind
`TrackResult`; deskew with per-point timestamps; new unit tests; bug fix with
a reproducer; launch/compose changes; GPS ENU anchoring once the fusion design
says where it plugs in.

**fable — reasoning-heavy or contract-changing.** Correctness depends on math
or system-level judgment, or the change moves a contract other components rely
on. Examples: preintegration covariance/Jacobians; fixed-lag smoother design;
degeneracy remediation; any `slam_interfaces/msg` change; QoS/topic contract
changes; cloud-boundary questions; cross-package refactors; any task whose
spec is ambiguous after reading STATE.md + code_graph.

### Routing algorithm (for the orchestrating session)

1. Map the task to activity IDs / components in `code_graph.json`.
2. Take the **max** `edit_tier` across every component the diff will touch.
3. Design/spec phases of split activities (`fable design → sonnet impl`) run
   at fable; the implementation subagent gets the written spec, not the problem.
4. Delegate with the Agent tool using an explicit `model` override; give the
   subagent: the activity definition, the files list from the code graph, the
   verification gate for its tier, and the escalation triggers verbatim.
5. Fable reviews (not rewrites) sonnet diffs that touch `review_required`
   components before push.

### Escalation triggers (hard, verbatim in every delegation prompt)

Stop and return to the orchestrator immediately if:
- your diff would touch `slam_core/` math or `slam_interfaces/msg/`;
- a published topic name, type, or QoS would change;
- the change would add any network dependency to the runtime;
- tests fail for reasons outside your task's scope;
- you have attempted the same fix twice without success;
- the task requires breaking a STATE.md invariant.

An escalation is a success condition, not a failure: a cheap model that stops
early costs less than a wrong merge.

### Verification gates per tier

| Tier | Must pass before handing back |
|---|---|
| haiku | `ctest --test-dir tests/build` green; black on touched `.py` |
| sonnet | haiku gate + `g++ -fsyntax-only` on touched ROS-free headers + Docker/colcon build when ROS node code changed |
| fable | sonnet gate + self-review of math against the cited reference (e.g. Forster RSS'15 equations) |

## 5. Cost model (why this pays)

The skeleton's extension points were designed so that ~70% of the remaining
work queue (see STATE.md) is sonnet-or-below: the contracts, residual layouts,
and health/state plumbing — the parts where a wrong decision is expensive —
are already fixed. Fable engagement concentrates on the four genuinely hard
items (A1, A4, A7, A8) and on reviewing math-adjacent diffs.
