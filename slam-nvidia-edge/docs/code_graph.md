# Code graph

Human-readable view of `code_graph.json` (that file is the machine-readable
source of truth for delegation — components, edit tiers, activity definitions).

## Package dependency graph

```mermaid
graph TD
    subgraph contracts [Contracts — fable-tier edits]
        SI[slam_interfaces<br/>msgs]
        SC[slam_core<br/>header-only math]
    end

    subgraph runtime [Runtime nodes — sonnet-tier edits]
        TF[tensor_frontend_node]
        VIO[vio_node]
        LIO[lio_node]
        FUS[fusion_node]
        MAP[map_node]
        HM[health_monitor_node]
        REC[recorder_node]
    end

    subgraph ops [Ops — haiku-tier edits]
        BR[slam_bringup]
        CFG[configs/]
        SCR[scripts/]
    end

    SC --> VIO
    SC --> LIO
    SC --> FUS
    SC --> HM
    SI --> VIO
    SI --> LIO
    SI --> FUS
    SI --> HM
    SI --> REC
    TF -. "/slam/frontend/features" .-> VIO
    VIO -- "/slam/vio/state" --> FUS
    LIO -- "/slam/lio/state" --> FUS
    LIO -- "/slam/lio/local_map" --> MAP
    VIO -- health --> HM
    LIO -- health --> HM
    FUS -- health --> HM
    FUS -- "/slam/fused/state" --> REC
    BR --> runtime
    CFG --> BR
```

Tier markings are defaults per package; `sub_tiers` in the JSON override them
per file (e.g. `sliding_window_estimator.hpp` inside sonnet-tier `vio_node` is
fable, because it is estimator math).

## Activity dependency graph

```mermaid
graph LR
    A12[A12 CI<br/>sonnet→haiku] --> A2[A2 Sophus swap<br/>sonnet]
    A1[A1 preint covariance<br/>fable] --> A4[A4 sliding window<br/>fable→sonnet]
    A1 --> A5[A5 real ICP<br/>sonnet +fable review]
    A1 --> A8[A8 EKF fusion<br/>fable→sonnet]
    A5 --> A7[A7 degeneracy remap<br/>fable]
    A5 --> A11[A11 persistent map<br/>sonnet]
    A8 --> A9[A9 GPS/wheel align<br/>sonnet]
    A3[A3 real tracker<br/>sonnet]
    A6[A6 deskew<br/>sonnet]
    A10[A10 TrtEngine<br/>sonnet]
```

Independent lanes (`A3`, `A6`, `A10`, `A12`) can run as parallel cheap-model
delegations today; the `A1` fanout is the fable-gated critical path.

## Reading order for a new session

1. `STATE.md` — where things stand.
2. This file — what depends on what.
3. The contract header(s) of your target component (each placeholder file
   marks its extension point with `EXTENSION POINT:` / `TODO(...)` comments).
