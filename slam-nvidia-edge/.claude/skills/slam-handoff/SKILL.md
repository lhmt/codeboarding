---
name: slam-handoff
description: Mandatory end-of-session procedure for slam-nvidia-edge — update STATE.md and write the handoff file before pushing.
---

# End-of-session handoff

Run this before the final push of any session that changed code or docs.

## 1. Update STATE.md

- **Component status**: adjust maturity for components you changed.
- **Work queue**: remove finished activities; add newly discovered ones with a
  tier tag (add their full definition to `docs/code_graph.json` too).
- **Last verification**: replace with what you actually ran today.
- Do not touch **Invariants** without fable-tier authority.
- Keep the whole file under ~100 lines — prune, don't append.

## 2. Write the handoff

Copy `handoffs/TEMPLATE.md` to `handoffs/YYYY-MM-DD-<slug>.md` and fill every
section. Handoffs are append-only: never edit a previous session's file.

Quality bar per section:
- **In flight**: name the exact file + function + what the next edit is. "The
  ICP is partially done" is useless; "resume at `point_to_plane_icp.hpp:align`,
  correspondence search done, GN update not wired" is a handoff.
- **Decisions**: only choices that a future session might plausibly reverse by
  accident. Include the *why*.
- **Verification**: paste command output, don't summarize it.

## 3. Consistency check

- STATE.md queue and code_graph.json activities agree.
- CLAUDE.md still accurate if you changed protocol/commands (rare; fable-tier).
- Commit STATE.md + handoff together with (or immediately after) the code
  commit they describe, on the same branch/PR.
