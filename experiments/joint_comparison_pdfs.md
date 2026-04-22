# Joint Comparison PDFs — Detailed Descriptions

These PDFs visualize CPG controller outputs over time for the spider and gecko robots
under different coupling topologies. They are produced by re-integrating the trained
CPG weight matrix using RK45 (no physics simulation), so they show what the controller
is *commanding*, not actual joint angles after physics interaction with the ground.

## Common Setup (all PDFs)

| Parameter | Value |
|---|---|
| Controller type | `ode_cpg` (Revolve2 CPG with RK45 integration) |
| Initial state | uniform `±√2/2 ≈ ±0.707` (Revolve2 default) |
| Output range | `[-1, 1]` (clipped after each integration step) |
| Control frequency | 20 Hz (600 timesteps over 30 s) |
| Simulation time | 30 s |
| Population (CMA-ES) | 25 |
| Generations (CMA-ES) | 300 |
| Penalty type | `dragging` (foot-aware) |
| λ shown in plots | 0 (no dragging penalty applied) |
| Run shown | median fitness run out of 30 runs |
| Fitness formula | `distance × (1 − dragging)^λ` |

The "median fitness run" is the run whose final-generation best fitness is the median
of all 30 runs for that configuration (sorted by fitness, then index 15 selected).

## Coupling Topologies

| Coupling | Description | Spider params | Gecko params |
|---|---|---|---|
| Uncoupled | Each oscillator independent (no coupling weights) | 8 | 6 |
| Neighbour | Connections within 2 module-tree hops | 12 | 9 |
| Structured (BLF) | Bio-inspired: hip-hip all-to-all + hip-knee + spine couplings | 18 | 17 |
| Fully connected | All pairs of oscillators coupled | 36 | 21 |

All CPG parameters consist of `n_hinges` internal weights (one per oscillator)
followed by coupling weights (one per connection). The internal weights create the
oscillation; the coupling weights create phase relationships between oscillators.

---

## `spider_joint_comparison.pdf`

A **4 × 4 grid** showing CPG joint outputs over 30 seconds for the **spider robot**
(8 hinges, 4 legs × 2 joints).

### Layout

- **Rows (4 couplings):** Uncoupled, Neighbour, Structured, Fully connected
- **Columns (4 legs):** Left, Right, Front, Back
- Each cell contains 2 lines:
  - **Hip:** solid blue (`#2266bb`)
  - **Knee:** dashed orange (`#dd6622`)
- X-axis: time in seconds (0–30 s)
- Y-axis: oscillator output `[-1, 1]` (shared across all subplots)

### Hinge → leg mapping (spider)

| Hinge index | Leg | Joint |
|---|---|---|
| 0 | Left | Hip |
| 1 | Left | Knee |
| 2 | Right | Hip |
| 3 | Right | Knee |
| 4 | Front | Hip |
| 5 | Front | Knee |
| 6 | Back | Hip |
| 7 | Back | Knee |

### Bounds and source

| Coupling | Internal bounds | Coupling bounds | Source dir | Median run |
|---|---|---|---|---|
| Uncoupled | `[-1, 1]` | (none) | `final_results/spider_ode_cpg_uncoupled_lambda0_dragging` | 24 |
| Neighbour | `[-1, 1]` | `[-1, 1]` | `final_results/spider_ode_cpg_neighbor_lambda0_dragging` | 26 |
| Structured | `[-1, 1]` | `[-1, 1]` | `final_results/spider_ode_cpg_blf_lambda0_dragging` | 9 |
| Fully connected | `[-1, 1]` | `[-1, 1]` | `fully_connected_local/spider_ode_cpg_fully_connected_lambda0_dragging` | 4 |

---

## `gecko_joint_comparison.pdf`

A **4 × 3 grid** showing CPG joint outputs over 30 seconds for the **gecko robot**
(6 hinges).

### Layout

- **Rows (4 couplings):** Uncoupled, Neighbour, Structured, Fully connected
- **Columns (3 anatomical groups):** Front legs / Spine / Rear legs
- Each cell contains 2 lines:
  - **Front legs column:** Front-L (solid blue) + Front-R (dashed orange)
  - **Spine column:** Spine 1 (solid blue) + Spine 2 (dashed orange)
  - **Rear legs column:** Rear-L (solid blue) + Rear-R (dashed orange)

### Hinge → anatomical mapping (gecko)

| Hinge index | Location |
|---|---|
| 0 | Front-left leg (single joint) |
| 1 | Front-right leg (single joint) |
| 2 | Spine 1 (closer to core) |
| 3 | Spine 2 (closer to rear) |
| 4 | Rear-left leg (single joint) |
| 5 | Rear-right leg (single joint) |

### Bounds and source

| Coupling | Internal bounds | Coupling bounds | Source dir | Median run |
|---|---|---|---|---|
| Uncoupled | `[-1, 1]` | (none) | `final_results/gecko_ode_cpg_uncoupled_lambda0_dragging` | 27 |
| Neighbour | `[-1, 1]` | `[-1, 1]` | `final_results/gecko_ode_cpg_neighbor_lambda0_dragging` | 1 |
| Structured | `[-1, 1]` | `[-1, 1]` | `final_results/gecko_ode_cpg_blf_lambda0_dragging` | 4 |
| Fully connected | `[-1, 1]` | `[-1, 1]` | `fully_connected_local/gecko_ode_cpg_fully_connected_lambda0_dragging` | 11 |

---

## `spider_joint_comparison_split.pdf`

Same layout as `spider_joint_comparison.pdf` but a **3 × 4 grid** (no fully-connected
row, since the split-bounds experiment did not include the fully-connected topology).

### Key difference: split bounds

This experiment uses **different bounds for internal vs coupling weights**:

- Internal weights (one per oscillator): `[-1, 1]`
- Coupling weights (one per connection): `[-2, 2]`

The motivation is that coupling weights drive the phase relationships between
oscillators and may benefit from a wider search range than the internal oscillator
weights. The internal weights set the amplitude and frequency of each individual
oscillator, while the coupling weights determine how oscillators influence each other.

### Layout

- **Rows (3 couplings):** Uncoupled, Neighbour, Structured
- **Columns (4 legs):** Left, Right, Front, Back
- Each cell: hip (solid blue) + knee (dashed orange)
- λ = 0 (no dragging penalty)

### Bounds and source

| Coupling | Internal bounds | Coupling bounds | Source dir | Median run |
|---|---|---|---|---|
| Uncoupled | `[-1, 1]` | (none) | `split_bounds_local/spider_ode_cpg_uncoupled_lambda0_dragging` | 24 |
| Neighbour | `[-1, 1]` | **`[-2, 2]`** | `split_bounds_local/spider_ode_cpg_neighbor_lambda0_dragging` | 9 |
| Structured | `[-1, 1]` | **`[-2, 2]`** | `split_bounds_local/spider_ode_cpg_blf_lambda0_dragging` | 4 |

For the uncoupled topology there are no coupling weights, so the split-bounds setting
has no effect — uncoupled is identical to the standard `final_results` uncoupled
experiment.

---

## `gecko_joint_comparison_split.pdf`

Same layout as `gecko_joint_comparison.pdf` but a **3 × 3 grid** (no fully-connected
row). Uses the same split-bounds setup as the spider split file:

- Internal weights: `[-1, 1]`
- Coupling weights: `[-2, 2]`

### Layout

- **Rows (3 couplings):** Uncoupled, Neighbour, Structured
- **Columns (3 anatomical groups):** Front legs / Spine / Rear legs
- Same line conventions as the main gecko plot

### Bounds and source

| Coupling | Internal bounds | Coupling bounds | Source dir | Median run |
|---|---|---|---|---|
| Uncoupled | `[-1, 1]` | (none) | `split_bounds_local/gecko_ode_cpg_uncoupled_lambda0_dragging` | 18 |
| Neighbour | `[-1, 1]` | **`[-2, 2]`** | `split_bounds_local/gecko_ode_cpg_neighbor_lambda0_dragging` | 30 |
| Structured | `[-1, 1]` | **`[-2, 2]`** | `split_bounds_local/gecko_ode_cpg_blf_lambda0_dragging` | 27 |

---

## How to reproduce

The plots are generated by these scripts (located in `revolve2/experiments/`):

| Script | Outputs |
|---|---|
| `plot_joint_comparison.py` | `spider_joint_comparison.pdf` |
| `plot_joint_comparison_gecko.py` | `gecko_joint_comparison.pdf` |
| `plot_joint_comparison_split.py` | `spider_joint_comparison_split.pdf`, `gecko_joint_comparison_split.pdf` |

Each script:

1. Loads `best_params_run_<median>.npy` for the median-fitness run
2. Builds the CPG network structure for the given coupling type
3. Constructs the weight matrix from the saved parameters
4. Initializes state to uniform `±√2/2`
5. Integrates `X' = WX` with RK45 for 600 steps at dt = 1/20 s
6. Plots the first `n_hinges` elements of the state vector (oscillator outputs)

The plots show **commanded controller outputs**, not real joint angles after physics
interaction with the environment. Two controllers with identical commanded outputs
may behave differently in simulation if their robot bodies interact with the ground
differently.
