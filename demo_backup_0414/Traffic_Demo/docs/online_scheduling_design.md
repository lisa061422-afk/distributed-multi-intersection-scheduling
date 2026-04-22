# Online Scheduling Design for 5-Intersection Traffic Demo

> Context: Current demo uses **offline scheduling** — the full trajectory for all vehicles is
> pre-computed by MATLAB before playback begins.  
> This document records design ideas for upgrading to **online (reactive) scheduling**,
> where the scheduler re-solves in response to mid-simulation disturbances.

---

## 1. Pedestrian Flow Model

Pedestrians are treated as **highest-priority, non-preemptive agents**:

| Property | Value |
|---|---|
| Priority | Highest (overrides all vehicles) |
| Preemption | None — once a pedestrian starts crossing, vehicles must wait |
| Time window | Pedestrian occupies crosswalk for duration `[t_enter, t_exit]` |
| Constraint type | Hard time-window exclusion for all vehicles using same intersection approach |

The scheduler must reserve a **conflict-free gap** in every vehicle's schedule around
each pedestrian crossing window.  
Vehicles delayed by a pedestrian may cascade and delay other downstream vehicles.

---

## 2. Online Scheduling Modes

### 2.1 Classification by Trigger Mechanism

| Mode | Trigger | Re-plan Scope | Suitable When |
|---|---|---|---|
| **Time-triggered MPC** | Fixed period `T` | Receding horizon | Steady-state, predictable load |
| **Event-triggered** | Specific event detected | Affected subset | Sparse, high-impact events |
| **State-triggered** | Threshold crossed (queue, delay) | Global or local | Congestion management |
| **Arrival-triggered** | New agent enters detection zone | From arrival onward | Intersection admission control |
| **Conflict-triggered** | New time-space conflict detected | Conflicting agents only | Minimal-disturbance replanning |
| **Phase-triggered** | Current phase completes | Next phase | Traffic-light-style sequencing |
| **Hybrid (event + time backup)** | Event primary; periodic fallback | Adaptive | Robust to missed events |

### 2.2 Feasibility with 15 s Computation Time

Each MATLAB re-solve currently takes **~15 seconds** wall-clock time.  
Vehicle traversal time per intersection: ~5–15 s.  
Detection zone lead time (approach → intersection): ~20–25 s.

| Mode | Feasible at 15 s? | Notes |
|---|---|---|
| High-frequency Time-triggered (T < 20 s) | ❌ | Computation finishes after the event has passed |
| Low-frequency Time-triggered (T = 30–60 s) | ⚠️ | Works but misses short-lived disturbances |
| **Arrival/Event-triggered with anticipatory detection** | ✅ | Trigger at ~20 s lead → plan ready with ~5 s buffer |
| Conflict-triggered (subset only) | ✅ | Smaller problem → faster; good for cascading delays |
| Hybrid event + periodic backup | ✅ | Most robust for demo |

### 2.3 Recommended Architecture (Demo)

```
Disturbance detected in approach zone  (~20-25 s lead time)
            │
            ▼
    Trigger MATLAB re-solve  (15 s computation)
            │
            ▼
    New schedule ready       (~5-10 s before conflict point)
            │
            ▼
    Push new .js schedule to demo → vehicles update trajectories
```

---

## 3. Interactive Disturbance Elements

The following elements can be **manually placed by the user** during playback.
Each creates a conflict with the current offline schedule; the online scheduler
resolves it and produces a new conflict-free plan.

### 3.1 Summary Table

| Element | Conflict Type | Visual Effect | Implementation Difficulty | Online Advantage |
|---|---|---|---|---|
| 🚶 **Pedestrian** | Time-window insertion at crosswalk | Vehicle stops at crosswalk, pedestrian crosses, vehicle resumes | Low (crosswalks already drawn) | Clear single-agent priority |
| 🚑 **Emergency Vehicle** | High-priority path reservation across multiple intersections | Multiple vehicles simultaneously yield | Medium | Dramatic multi-vehicle cascade |
| 🚗 **Stalled Vehicle** | Intersection blocked for `T` extra seconds | Downstream vehicles queue and wait | Low | Visible cascading delay |
| 🚧 **Road Closure** | Road segment unavailable for `T` seconds | Vehicles wait for reopening | Medium | Rescheduling under topology change |

### 3.2 Pedestrian (🚶)

**Trigger:** User clicks any crosswalk on the map → places a pedestrian.

**Conflict mechanism:**
```
Original schedule : vehicle crosses P2-North at t = 8 s
User places pedestrian : occupies P2-North crosswalk t ∈ [7, 10] s
Conflict          : vehicle arrival window ∩ pedestrian window ≠ ∅
Online resolution : delay vehicle to t ≥ 10 s; propagate to downstream vehicles
```

**MATLAB input interface:**
```
pedestrian_event.intersection = 'P2';
pedestrian_event.approach     = 'North';   % N / S / E / W
pedestrian_event.t_enter      = 7.0;       % s, from sim start
pedestrian_event.t_duration   = 3.0;       % s
```

### 3.3 Emergency Vehicle (🚑)

**Trigger:** User places an emergency vehicle at any port.

**Conflict mechanism:**
- Emergency vehicle claims absolute priority along its full path
- All vehicles whose schedule overlaps any intersection on that path must yield
- Can affect 2–4 intersections simultaneously

**MATLAB input interface:**
```
emergency.entry_port  = 3;           % port number
emergency.path        = ['P2','P4']; % intersections traversed
emergency.t_enter     = 5.0;        % s
emergency.speed_ratio = 1.5;        % relative to normal vehicle
```

### 3.4 Stalled Vehicle (🚗)

**Trigger:** User clicks a moving vehicle → marks it as stalled for `T` seconds.

**Conflict mechanism:**
- Stalled vehicle occupies an intersection for `T` extra seconds beyond its scheduled window
- All vehicles with scheduled arrival after the stall but before `t_resume` are delayed
- Cascading effect: delays at one intersection propagate to downstream intersections

**MATLAB input interface:**
```
stall_event.vehicle_id  = 'V3';
stall_event.t_start     = 6.0;   % s, when stall begins
stall_event.duration    = 8.0;   % s, extra occupancy time
```

### 3.5 Road Closure (🚧)

**Trigger:** User clicks a road segment (R6–R10) to close it for `T` seconds.

**Conflict mechanism:**
- Road becomes unavailable during `[t_close, t_reopen]`
- Vehicles scheduled to use that segment in that window must wait

**MATLAB input interface:**
```
closure.road_id  = 'R6';
closure.t_close  = 4.0;    % s
closure.duration = 15.0;   % s
```

---

## 4. Demo Display Design

### 4.1 Single-Panel Online Demo Flow

```
Simulation playing (offline schedule) 
        │
        │  User places disturbance
        ▼
⚠ "Conflict Detected" banner appears
  Affected vehicles highlighted in red
        │
        ▼
"Replanning…" progress bar (simulates 15 s computation)
        │
        ▼
✓ "Schedule Updated"
  Vehicle trajectories update in real-time
  Delay delta shown: e.g. "+2.3 s total delay"
        │
        ▼
Simulation resumes with new conflict-free schedule
```

### 4.2 Side-by-Side Comparison Panel (stronger demo)

| Left: Offline (no reaction) | Right: Online (re-scheduled) |
|---|---|
| Disturbance ignored | Disturbance detected and handled |
| Conflict highlighted (red flash) | No conflict — smooth crossing |
| Total Delay Cost unchanged | Total Delay Cost increases minimally |
| Vehicles "collide" with pedestrian | Vehicle waits, pedestrian crosses safely |

### 4.3 Metrics to Display

| Metric | Description |
|---|---|
| **Total Delay Cost** | Accumulated delay penalty (already implemented) |
| **Δ Delay vs Offline** | Extra delay introduced by disturbance handling |
| **Vehicles Affected** | Number of vehicles whose schedule changed |
| **Replan Trigger Count** | How many times online re-solve was triggered |
| **Time to Replan** | Wall-clock computation time (show as progress bar) |

---

## 5. Phased Implementation Roadmap

| Phase | Description | Effort |
|---|---|---|
| **Phase 1** (now feasible) | Pre-compute a lookup table of scenarios with different disturbances; demo loads the matching `.js` file when user places a disturbance. Looks online, no real-time computation needed. | Low |
| **Phase 2** | MATLAB script accepts a `disturbance_event` struct, re-runs the optimizer, outputs a new schedule `.js`. User triggers this manually; 15 s pause in demo. | Medium |
| **Phase 3** | Background MATLAB process listens for events from browser (via local HTTP or WebSocket); true real-time online scheduling. | High |

**Recommended starting point:** Phase 1 with Pedestrian + Stalled Vehicle,
using 3–4 pre-computed "disturbance scenarios" per intersection.
This delivers a compelling online-vs-offline visual comparison
without requiring real-time computation infrastructure.

---

## 6. Key Design Principle

> **The demo's goal is not to show that online scheduling is fast —  
> it is to show that online scheduling finds a solution that offline scheduling cannot.**
>
> The most compelling metric: given the *same* disturbance,  
> online scheduling achieves **lower total delay** than offline (which either ignores  
> the disturbance and collides, or over-reserves time and wastes capacity).
