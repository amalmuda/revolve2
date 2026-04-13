# Body Part Identification and Structured Coupling Rules

## Overview

These rules classify every module in a modular robot's body tree and determine how CPG oscillators are coupled. They are adapted from the Body/Limb Finder (BLF) algorithm by Bonardi et al. (2014), with modifications for tree-structured bodies in Revolve2.

Two systems work together:

1. **Body Part Identification** -- classifies modules as body/limb/foot and hinges as spine/hip/knee
2. **Structured Coupling** -- determines which CPG oscillators are connected, based on the hinge classifications

---

## Body Part Identification Rules

### Rule 1: Core is always body

The core module is always classified as body, regardless of how many children it has.

### Rule 2: Brick clusters with more than 2 hinges are body

A connected group of bricks (a brick cluster) is classified as body if more than 2 active hinges are directly attached to it. This identifies junction points where multiple limbs or branches meet.

- 2 hinges touching a brick cluster = chain spacer (not body)
- 3+ hinges touching a brick cluster = junction between branches (body)

A "brick cluster" is any connected group of non-hinge, non-core modules. A single brick counts as a cluster of size 1. "Directly attached" means the hinge is an immediate neighbor of any brick in the cluster.

**Example -- spider leg spacer:** `hip -> brick -> hinge` -- brick has 2 hinges touching it (one on each side), so it is NOT body. It is just a spacer in a limb chain.

**Example -- gecko end brick:** The rear brick of the spine has 3 hinges touching it (spine hinge from front + left leg hip + right leg hip) -> body by Rule 2.

**Example -- gecko intermediate brick:** The brick between the two spine hinges has only 2 hinges touching it (spine1 from front + spine2 going forward) -> NOT body by Rule 2. But see Rule 3.

**Example -- queen T-cluster:** A connected group of 4 bricks with 5 hinges touching it -> body.

### Rule 3: Modules between body regions are body

Any module on the shortest path between two body regions (identified by rules 1 and 2) is also classified as body. This captures spine joints and intermediate bricks that connect the core to distant body brick clusters.

This is where the intermediate modules in a spine get classified. Rule 2 identifies the "anchor points" (core + junction bricks), and Rule 3 fills in everything between them.

**Example -- gecko:** Core is body (Rule 1). The rear brick is body (Rule 2, 3 hinges touch it). The shortest path between them is: Core -> spine hinge 1 -> intermediate brick -> spine hinge 2 -> rear brick. Rule 3 classifies spine hinge 1, the intermediate brick, and spine hinge 2 as body. This is how the full spine gets detected even though the intermediate brick only has 2 touching hinges.

**Example -- queen:** Core is body (Rule 1). The T-shaped brick cluster is body (Rule 2, 5 hinges). The path from core to brick cluster goes through 3 hinges -> all become body (spine).

### Rule 4: Everything else is a limb

Any module not classified as body by rules 1-3 forms part of a limb. Each connected group of non-body modules branching off the body is a separate limb with its own limb ID.

### Rule 5: Hinge classification

Active hinges are classified based on their position:

- **Spine**: any active hinge classified as body (by rules 1-3)
- **Hip**: the first active hinge in each limb (closest to the body)
- **Knee**: all remaining active hinges after the hip in the same limb

No distinction is made between knee and ankle -- all post-hip hinges in a limb are knees. This simplification does not affect coupling, since the coupling within a limb is always a chain regardless of labeling. The code uses `JointType.KNEE` for all post-hip hinges.

### Rule 6: Foot detection

A module is a **foot** if:
- (a) It has a hinge somewhere in its ancestor chain (including itself) -- it is part of a limb, not the core/body region
- (b) It has NO hinge as a descendant -- it is past the last actuated joint

In other words: **everything after the last hinge in a chain is foot.**

This means:
- Blob of bricks after last hinge -> all foot
- Bare hinge ending (no brick after it) -> the hinge itself is the foot
- Core -> not foot
- Bricks attached to core with no hinge ancestor -> not foot

Foot detection is used for the dragging penalty in the fitness function. Only foot modules may touch the ground without penalty.

---

## Structured Coupling Rules

### Spine coupling: all-to-all

All spine oscillators are coupled to each other. If there are S spine joints, this adds S x (S-1) / 2 coupling connections. This allows global coordination of the body, regardless of joint orientation (horizontal, vertical, or mixed).

### Hip coupling: all-to-all + nearest spine

All hip oscillators are coupled to each other. If there are H hips, this adds H x (H-1) / 2 connections.

Additionally, each hip is coupled to its **nearest spine joint** (by shortest path through the module tree). If a hip is equidistant to two spine joints, it connects to both.

If there are no spine joints (e.g., spider), hips are only coupled all-to-all with each other.

### Knee coupling: chain within limb

Each knee is coupled to the **previous hinge in the same limb** (the one closer to the body). This creates a chain:

```
hip <- knee1 <- knee2 <- knee3 ...
```

Knees are not coupled across limbs.

---

## Design Decisions and Differences from Bonardi (2014)

| Aspect | Bonardi (2014) | Our approach |
|---|---|---|
| Robot structure | Cyclic graphs | Trees (Revolve2) |
| Core classification | Via articulation points | Always body (Rule 1) |
| Body detection | Bi-connected components + articulation points | Brick cluster hinge count (Rule 2) + path filling (Rule 3) |
| Spine joints | Only 1 per linear body segment (most central) | All body hinges are spine |
| Locked joints | Unclassified joints are locked (0 amplitude) | All hinges active, nothing locked |
| Knee/ankle distinction | Separate categories (1 knee, 1 ankle per limb) | All post-hip hinges are knees |
| Spine coupling | All-to-all | All-to-all (same) |
| Hip coupling | All-to-all + nearest spine | All-to-all + nearest spine, ties connect to both |
| Symmetry (BLF-SYM) | Symmetric limbs share parameters | Not used |
| Foot detection | Not defined (different fitness) | Everything after last hinge in chain (Rule 6) |

### Key justifications

- **Brick cluster rule instead of articulation points**: Bonardi's algorithm uses articulation points and bi-connected components for cyclic graphs. Our robots are trees (no cycles), so articulation points reduce to simple degree checks. The brick cluster hinge count rule is simpler, more intuitive, and produces the same result for tree structures: it identifies junction points where 3+ branches meet.

- **All hinges active**: Bonardi locks unclassified joints (sets amplitude to 0). We keep all hinges active because CMA-ES handles higher dimensions well. If a joint shouldn't move, evolution can set its weight to ~0 naturally.

- **All body hinges are spine**: Bonardi activates only the most central hinge per linear body segment. We activate all of them because (a) our robots are small enough that the extra parameters don't hurt CMA-ES, and (b) it allows richer spine undulation with mixed horizontal/vertical joints.

- **Spine all-to-all**: preserves global body coordination, especially when spine joints are in different planes (horizontal + vertical). Bonardi used this too.

- **No knee/ankle distinction**: coupling within a limb is always a chain from hip outward. Whether you call the last hinge "ankle" or "knee3" doesn't change the coupling connections. Simplifies the code.

- **No symmetry sharing**: minimal benefit for asymmetric robots like queen or salamander. CMA-ES handles the extra parameters. Could be added later if needed.

---

## Classification Examples

### Spider (8 hinges)

**Rule 2 check:** Each brick in a spider leg sits between exactly 2 hinges (e.g., `hip -> brick -> knee`). No brick cluster has >2 touching hinges. Rule 2 does not trigger.

**Result:**
- Body: core only (Rule 1)
- Spine: none (no body hinges)
- Limbs: 4 legs, each `hip -> brick -> knee -> brick`
- Classification: 0 spine, 4 hips, 4 knees

**Coupling:**
- Hip all-to-all: 4 x 3 / 2 = 6 connections
- No hip-spine (no spine exists)
- Knee chain: 4 connections (each knee to its hip, one per limb)
- Total coupling: 6 + 4 = 10

**Total params: 8 internal + 10 coupling = 18**

### Gecko (6 hinges)

Structure: Core -> spine hinge 1 (90 deg) -> intermediate brick -> spine hinge 2 (90 deg) -> rear brick -> left/right rear legs. Core also has left/right front legs.

**Rule 2 check:**
- Intermediate brick: 2 touching hinges (spine1 + spine2) -> not body
- Rear brick: 3 touching hinges (spine2 + left leg hip + right leg hip) -> **body**
- Front leg bricks: 1 touching hinge each -> not body

**Rule 3:** Path from core (body, Rule 1) to rear brick (body, Rule 2): Core -> spine hinge 1 -> intermediate brick -> spine hinge 2 -> rear brick. All modules on this path become body. The intermediate brick and both spine hinges are now body.

**Result:**
- Body: core + intermediate brick + rear brick + spine hinge 1 + spine hinge 2 (5 nodes)
- Spine: 2 (spine hinge 1 and spine hinge 2)
- Limbs: 4 single-hinge legs (2 from core, 2 from rear brick), each just `hip -> brick`
- Classification: 2 spine, 4 hips, 0 knees

**Coupling:**
- Spine all-to-all: 1 connection
- Hip all-to-all: 4 x 3 / 2 = 6 connections
- Hip-to-nearest-spine: 4 connections (each hip to nearest spine via BFS)
- Total coupling: 1 + 6 + 4 = 11

**Total params: 6 internal + 11 coupling = 17**

### Queen (9 hinges)

Structure: Core -> 3-hinge spine chain -> T-shaped brick cluster (4 connected bricks) -> 5 limbs branching off. Core also has a single-hinge stub out back.

**Rule 2 check:**
- T-shaped brick cluster: 4 bricks connected together, 5 hinges directly touch it -> **body**
- No other brick cluster has >2 touching hinges

**Rule 3:** Path from core (body, Rule 1) to T-cluster (body, Rule 2) passes through 3 hinges -> all become body (spine).

**Result:**
- Body: core + 3 spine hinges + 4 bricks in T-cluster (8 nodes)
- Spine: 3
- Limbs: 5 (including the back stub). Limb 2 has hip + knee, rest have hip only.
- Classification: 3 spine, 5 hips, 1 knee

**Coupling:**
- Spine all-to-all: 3 x 2 / 2 = 3 connections
- Hip all-to-all: 5 x 4 / 2 = 10 connections
- Hip-to-nearest-spine: 5 connections
- Knee chain: 1 connection (knee to its hip)
- Total coupling: 3 + 10 + 5 + 1 = 19

**Total params: 9 internal + 19 coupling = 28**

### Ant (8 hinges)

Structure: Core -> spine hinge 1 -> brick (with left/right mid legs) -> spine hinge 2 -> brick (with left/right rear legs). Core also has left/right front legs.

**Rule 2 check:**
- Spine 1 brick: 4 touching hinges (spine1 + left mid hip + right mid hip + spine2) -> **body**
- Spine 2 brick: 3 touching hinges (spine2 + left rear hip + right rear hip) -> **body**

**Rule 3:** Path from core to spine 1 brick passes through spine hinge 1 -> body. Path from spine 1 brick to spine 2 brick passes through spine hinge 2 -> body (already covered).

**Result:**
- Body: core + 2 spine hinges + 2 bricks (5 nodes)
- Spine: 2
- Limbs: 6 single-hinge legs
- Classification: 2 spine, 6 hips, 0 knees

**Coupling:**
- Spine all-to-all: 1 connection
- Hip all-to-all: 6 x 5 / 2 = 15 connections
- Hip-to-nearest-spine: 8 connections (front legs -> spine 1, rear legs -> spine 2, middle legs are equidistant to both spines so connect to both)
- Total coupling: 1 + 15 + 8 = 24

**Total params: 8 internal + 24 coupling = 32**

### Snake (8 hinges)

Structure: Core -> hinge -> brick -> hinge -> brick -> ... (pure linear chain, no branching)

**Rule 2 check:** Every brick has exactly 2 touching hinges (one on each side). No brick has >2. Rule 2 never triggers.

**Rule 3:** Only one body region (core), so no paths to fill.

**Result:**
- Body: core only
- Spine: none
- Limbs: 1 single limb containing all 15 non-core modules
- Classification: 0 spine, 1 hip (first hinge), 7 knees (rest of chain)
- This is the **degenerate case** -- BLF cannot identify a spine because there is no branching at all. The entire robot is one limb.

**Coupling:**
- Hip all-to-all: only 1 hip, no connections
- Knee chain: 7 connections (each knee to previous hinge)
- Total coupling: 7

**Total params: 8 internal + 7 coupling = 15**

---

## Implementation

- **Body part identification**: `BodyLimbFinder` class in `blf.py`
- **Coupling generation**: `BLFCpgNetworkGenerator.generate()` in `blf.py`
- **Foot detection**: `identify_geometry_types()` in `contact_detection.py`
- **Entry point**: `active_hinges_to_cpg_network_structure_blf()` in `contact_detection.py`
