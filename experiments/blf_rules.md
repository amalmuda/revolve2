# Body Part Identification and Structured Coupling Rules

## Overview

These rules classify every module in a modular robot's body tree and determine how CPG oscillators are coupled. They are adapted from the Body/Limb Finder (BLF) algorithm by Bonardi et al. (2014), with modifications for tree-structured bodies in Revolve2.

Two systems work together:

1. **Body Part Identification** — classifies modules as body/limb/foot and hinges as spine/hip/knee
2. **Structured Coupling** — determines which CPG oscillators are connected, based on the hinge classifications

---

## Body Part Identification Rules

### Rule 1: Core is always body

The core module is always classified as body, regardless of how many children it has.

### Rule 2: Brick clusters with more than 2 hinges are body

A connected group of bricks (a brick cluster) is classified as body if more than 2 active hinges are directly attached to it. This identifies junction points where multiple limbs or branches meet.

- 2 hinges touching a brick = chain spacer (not body)
- 3+ hinges touching a brick cluster = junction between branches (body)

**Example — spider leg spacer:** `hip → brick → knee` — brick has 2 hinges, not body.

**Example — queen T-cluster:** brick blob with 5 hinges touching it → body.

### Rule 3: Modules between body regions are body

Any module on the path between two body regions (identified by rules 1 and 2) is also classified as body. This captures spine joints and intermediate bricks connecting the core to a distant brick cluster.

**Example — queen:** Core (body) → M2 → M3 → M4 → brick cluster (body). The three hinges and any bricks on this path are all body.

### Rule 4: Everything else is a limb

Any module not classified as body by rules 1–3 forms part of a limb. Each connected group of non-body modules branching off the body is a separate limb with its own limb ID.

### Rule 5: Hinge classification

Active hinges are classified based on their position:

- **Spine**: any active hinge classified as body (by rules 1–3)
- **Hip**: the first active hinge in each limb (closest to the body)
- **Knee**: all remaining active hinges after the hip in the same limb

No distinction is made between knee and ankle — all post-hip hinges are knees. This simplification does not affect coupling, since the coupling within a limb is always a chain regardless of labeling.

### Rule 6: Foot detection

A module is a **foot** if:
- (a) It has a hinge somewhere in its ancestor chain (including itself) — it is part of a limb, not the core/body region
- (b) It has NO hinge as a descendant — it is past the last actuated joint

In other words: **everything after the last hinge in a chain is foot.**

This means:
- Blob of bricks after last hinge → all foot
- Bare hinge ending (no brick after it) → foot (the hinge itself is the foot)
- Core → not foot
- Bricks on core with no hinge ancestor → not foot

Foot detection is used for the dragging penalty in the fitness function. Only foot modules may touch the ground without penalty.

---

## Structured Coupling Rules

### Spine coupling: all-to-all

All spine oscillators are coupled to each other. If there are S spine joints, this adds S×(S−1)/2 coupling connections. This allows global coordination of the body, regardless of joint orientation (horizontal, vertical, or mixed).

### Hip coupling: all-to-all + nearest spine

All hip oscillators are coupled to each other. If there are H hips, this adds H×(H−1)/2 connections.

Additionally, each hip is coupled to its **nearest spine joint** (by shortest path through the module tree). If a hip is equidistant to two spine joints, it connects to both.

If there are no spine joints (e.g., spider), hips are only coupled all-to-all with each other.

### Knee coupling: chain within limb

Each knee is coupled to the **previous hinge in the same limb** (the one closer to the body). This creates a chain:

```
hip ← knee1 ← knee2 ← knee3 ...
```

Knees are not coupled across limbs.

---

## Design Decisions and Differences from Bonardi (2014)

| Aspect | Bonardi (2014) | Our approach |
|---|---|---|
| Robot structure | Cyclic graphs | Trees (Revolve2) |
| Core classification | Via articulation points | Always body (rule 1) |
| Body detection | Bi-connected components + articulation points | Brick cluster hinge count + path rule |
| Spine joints | Only 1 per linear body segment (most central) | All body hinges are spine |
| Locked joints | Unclassified joints are locked | All hinges active, nothing locked |
| Knee/ankle distinction | Separate categories (1 knee, 1 ankle per limb) | All post-hip hinges are knees |
| Spine coupling | All-to-all | All-to-all (same) |
| Hip coupling | All-to-all + nearest spine | All-to-all + nearest spine, ties connect to both |
| Symmetry (BLF-SYM) | Symmetric limbs share parameters | Not used |
| Foot detection | Not defined (different fitness) | Everything after last hinge in chain |

### Key justifications:

- **All hinges active**: CMA-ES handles higher dimensions well. If a joint shouldn't move, evolution can set its weight to ~0.
- **Spine all-to-all**: preserves global body coordination, especially when spine joints are in different planes (horizontal + vertical). Bonardi used this too.
- **No knee/ankle distinction**: coupling within a limb is always a chain — the labeling doesn't change the structure.
- **No symmetry sharing**: minimal benefit for asymmetric robots like queen. CMA-ES handles the extra parameters.
- **Brick cluster rule**: replaces Bonardi's articulation point algorithm with a simpler rule suited to tree structures.

---

## Classification Examples

### Spider (8 hinges)

- Body: core only
- Spine: none
- Limbs: 4 legs × (hip + knee) = 4 hips + 4 knees
- Coupling: 6 hip-hip + 4 knee-hip = 10 connections
- **Total params: 8 internal + 10 coupling = 18**

### Gecko (6 hinges)

- Body: core + 2 spine hinges + bricks between core and rear branching brick
- Spine: 2 (all-to-all = 1 connection)
- Limbs: 4 legs × 1 hip = 4 hips
- Coupling: 1 spine-spine + 6 hip-hip + 4 hip-spine = 11 connections
- **Total params: 6 internal + 11 coupling = 17**

### Queen (9 hinges)

- Body: core + 3 spine hinges + T-shaped brick cluster
- Spine: 3 (all-to-all = 3 connections)
- Limbs: 5 (limb 0: 1 hip, limb 1: 1 hip, limb 2: 1 hip + 1 knee, limb 3: 1 hip, limb 4: 1 hip)
- Coupling: 3 spine-spine + 10 hip-hip + 5 hip-spine + 1 knee-hip = 19 connections
- **Total params: 9 internal + 19 coupling = 28**

---

## Implementation

- **Foot detection**: `identify_geometry_types()` in `contact_detection.py`
- **BLF classification**: `BodyLimbFinder` class in `blf.py`
- **Coupling generation**: `BLFCpgNetworkGenerator.generate()` in `blf.py`
- **Entry point**: `active_hinges_to_cpg_network_structure_blf()` in `contact_detection.py`
