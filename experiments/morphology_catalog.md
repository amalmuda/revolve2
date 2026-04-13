# Revolve2 Original Robot Morphology Catalog

All 22 robots from the upstream ci-group/revolve2 `modular_robots_v1` standards library.

## Module types

- **Core** (red in MuJoCo, 4 slots: front/back/left/right)
- **ActiveHinge** (white, 1-DoF revolute joint, 1 slot: attachment)
- **Brick** (blue, passive structural block, 3 slots: front/left/right)

## Hinge orientation convention

- **90°** — joint axis vertical, attached part swings horizontally (left-right)
- **0°** — joint axis horizontal, attached part swings vertically (up-down)
- **-90°** — like 90° but reversed initial phase

## BLF classification rules (new)

Body detection uses brick-cluster hinge counting (see blf_rules.md):
1. Core is always body
2. A connected brick cluster with >2 hinges touching it is body (junction point)
3. All modules on paths between body regions are body (spine filling)
4. Everything else is limb

Joint classification: body hinges = **spine**, first hinge in each limb = **hip**, rest = chain (no knee/ankle distinction — coupling is always a chain within a limb).

Coupling: spine all-to-all, hips all-to-all + nearest spine, limb hinges chained to previous hinge.

---

## 1. Spider (8 hinges)

**Shape:** Symmetric quadruped. 4 identical legs from core, no spine.

**Structure:** Each leg from core.{front,back,left,right}: `Hip(90°) → Brick → Hinge(0°) → Brick`

**Hinge orientations:** Hips at 90° (horizontal swing), second hinges at 0° (vertical bend).

**BLF:** Body = core only (no brick cluster has >2 hinges). 4 limbs × 2 hinges. 0 spine, 4 hips. 1 symmetric group (all 4 legs identical).

**CPG:** 8 internal + 10 coupling = **18 params**

**Modules:** 17 (1 core, 8 hinges, 8 bricks)

---

## 2. Gecko (6 hinges)

**Shape:** Elongated quadruped. 2-segment spine with 4 single-hinge legs.

**Structure:**
- Core → back: `Hinge(90°) → Brick → Hinge(90°) → Brick` (spine, 2 segments)
- Core → left/right: `Hip(0°) → Brick` (front legs)
- Spine end brick → left/right: `Hip(0°) → Brick` (rear legs)

**Hinge orientations:** Spine hinges at 90° (horizontal undulation). All leg hips at 0° (vertical paddle).

**BLF:** Spine end brick has 3 touching hinges (spine2 + 2 rear legs) → body by Rule 2. Path from core to end brick fills in spine. Body = core + 2 spine hinges + 2 bricks. 4 limbs × 1 hip each. 2 spine, 4 hips.

**CPG:** 6 internal + 11 coupling = **17 params**

**Modules:** 13 (1 core, 6 hinges, 6 bricks)

---

## 3. Babya (8 hinges)

**Shape:** Asymmetric gecko variant. Same spine + rear legs as gecko, but one front leg has 3 hinges while the other has 1.

**Structure:**
- Core → left: `Hip(0°) → Brick` (1-hinge leg)
- Core → right: `Hip(0°) → Hinge(90°) → Brick → Hinge(0°) → Brick` (3-hinge leg)
- Core → back: `Hinge(90°) → Brick → Hinge(90°) → Brick` (spine)
- Spine end brick → left/right: `Hip(0°) → Brick` (rear legs)

**Hinge orientations:** Spine at 90°. 3-hinge leg alternates 0°/90°/0°. Single-hinge legs at 0°.

**BLF:** Same as gecko — spine end brick has 3 touching hinges → body. 4 limbs (sizes: 2, 5, 2, 2 modules). 2 spine, 4 hips, 2 chain hinges.

**CPG:** 8 internal + 13 coupling = **21 params**

**Modules:** 16 (1 core, 8 hinges, 7 bricks)

---

## 4. Babyb (10 hinges)

**Shape:** Spider variant. 3 legs with 3 hinges each, 1 short leg with 1 hinge.

**Structure:**
- Core → left/right/front: `Hip(90°) → Brick → Hinge(0°) → Brick → Hinge(90°) → Brick` (3-hinge legs)
- Core → back: `Hip(90°) → Brick` (1-hinge leg)

**Hinge orientations:** 3-hinge legs alternate 90°/0°/90°. Short leg at 90°.

**BLF:** Body = core only (all brick spacers have exactly 2 touching hinges). 4 limbs. 0 spine, 4 hips, 6 chain hinges.

**CPG:** 10 internal + 12 coupling = **22 params**

**Modules:** 21 (1 core, 10 hinges, 10 bricks)

---

## 5. Ant (8 hinges)

**Shape:** Hexapod. 2-segment spine with 3 pairs of single-hinge legs.

**Structure:**
- Core → left/right: `Hip(0°) → Brick` (front legs)
- Core → back: `Hinge(90°) → Brick` (spine 1) → left/right: `Hip(0°) → Brick` (mid legs) → front: `Hinge(90°) → Brick` (spine 2) → left/right: `Hip(0°) → Brick` (rear legs)

**Hinge orientations:** Spine at 90° (horizontal undulation). All 6 leg hips at 0° (vertical swing).

**BLF:** Spine 1 brick has 4 touching hinges (spine1 + left + right + spine2) → body. Spine 2 brick has 3 (spine2 + left + right) → body. Path fills spine. 6 limbs × 1 hip. 2 spine, 6 hips.

**CPG:** 8 internal + 24 coupling = **32 params**

**Modules:** 17 (1 core, 8 hinges, 8 bricks)

---

## 6. Salamander (14 hinges)

**Shape:** Long-bodied amphibian. Long spine with many small appendages. Most complex morphology.

**Structure:** Core branches 3 ways: left (`Hinge(90°) → Hinge(-90°)`), right (`Hinge(0°)`), back (long spine chain through multiple bricks with side-branching hinges at intervals). Spine hinges at 90°, side appendage hinges at 0°, two sub-chain hinges at -90°.

**Hinge orientations:** Six at 90°, six at 0°, two at -90°.

**BLF:** Two brick clusters trigger Rule 2: one 2-brick cluster with 4 touching hinges, one 4-brick cluster with 5 touching hinges. Path rule fills spine between them and core. Body = 9 nodes. 8 limbs (many single-hinge stubs). 2 spine, 8 hips, 4 chain hinges.

**CPG:** 14 internal + 41 coupling = **55 params**

**Modules:** 25 (1 core, 14 hinges, 10 bricks)

---

## 7. Blokky (5 hinges)

**Shape:** Compact blob. Large brick mass with minimal actuation.

**Structure:**
- Core → left: `Hinge(90°)` (bare, no brick)
- Core → back: Brick → right: `Hinge(90°)`, front: `Hinge(90°) → Hinge(-90°) → Brick` → 7 bricks in a blob with 1 trailing `Hinge(0°)`

**Hinge orientations:** Three at 90°, one at -90°, one at 0°.

**BLF:** No brick cluster has >2 touching hinges (the 8-brick blob only touches 2 hinges). Body = core only. 2 limbs. 0 spine, 2 hips, 3 chain hinges. Lowest hinge count (5).

**CPG:** 5 internal + 4 coupling = **9 params**

**Modules:** 15 (1 core, 5 hinges, 9 bricks)

---

## 8. Park (8 hinges)

**Shape:** Spine with multiple small branches at two branching bricks.

**Structure:**
- Core → back: `Hinge(90°) → Hinge(-90°) → Brick` (spine start) → branches at two brick junctions with hinges at 0° and -90°

**Hinge orientations:** Spine start 90°/-90°. Branch hinges alternate 0° and -90°.

**BLF:** 3-brick cluster with 5 touching hinges → body. 2-brick cluster with 3 touching → body. Path fills spine. Body = 9 nodes. 5 limbs. 3 spine, 5 hips.

**CPG:** 8 internal + 18 coupling = **26 params**

**Modules:** 15 (1 core, 8 hinges, 6 bricks)

---

## 9. Garrix (10 hinges)

**Shape:** Asymmetric L-shape. Long hinge-chain spine with one 3-hinge limb.

**Structure:**
- Core → front: `Hinge(90°)` (stub)
- Core → left: `Hinge(90°) → Hinge(0°) → Hinge(-90°) → Brick` (3-hinge spine chain) → brick.left: `Hinge(0°) → Brick` (branching) → right: `Hinge(90°)`, front: `Hinge(90°)`, left: `Hinge(0°) → Hinge(90°) → Hinge(-90°) → Brick` (3-hinge limb)

**Hinge orientations:** Spine chain 90°/0°/-90°. Branch hinges 90°. Long limb 0°/90°/-90°.

**BLF:** Branching brick has 4 touching hinges → body. Path fills spine from core. Body = 7 nodes. 5 limbs (1 is a passive brick). 4 spine, 4 hips, 2 chain hinges.

**CPG:** 10 internal + 18 coupling = **28 params**

**Modules:** 15 (1 core, 10 hinges, 4 bricks)

---

## 10. Insect (9 hinges)

**Shape:** Compact, highly actuated. One 3-hinge chain limb, several short legs.

**Structure:**
- Core → right: `Hinge(90°) → Hinge(-90°) → Brick` (spine) → right: `Hinge(0°)`, front: `Hinge(90°)`, left: `Hinge(90°) → Brick` (more spine) → front: `Hinge(90°)`, right: `Hinge(0°) → Hinge(0°) → Hinge(90°)` (3-hinge chain)

**Hinge orientations:** Spine 90°/-90°. Short branches mix 0° and 90°. 3-hinge chain: 0°/0°/90°.

**BLF:** Both bricks have >2 touching hinges → body. Path fills spine. Body = 6 nodes. 4 limbs. 3 spine, 4 hips, 2 chain hinges. Highest hinge-to-module ratio (9/12 = 75%).

**CPG:** 9 internal + 17 coupling = **26 params**

**Modules:** 12 (1 core, 9 hinges, 2 bricks)

---

## 11. Linkin (12 hinges)

**Shape:** Chain/tentacle. Long spine with asymmetric branches. Fully asymmetric.

**Structure:**
- Core → back: `Hinge(0°)` (stub)
- Core → right: `Hinge(90°) → Hinge(0°) → Hinge(0°) → Hinge(-90°) → Brick` (4-hinge spine chain) → brick.front: Brick, brick.left: `Hip(0°) → Hinge(0°)` (2-hinge limb), brick.right: `Hip(90°) → Hinge(-90°) → Hinge(0°) → Hinge(90°) → Hinge(0°)` (5-hinge chain)

**Hinge orientations:** Spine 90°/0°/0°/-90°. Long limb 90°/-90°/0°/90°/0°. Stub 0°.

**BLF:** 2-brick cluster with 3 touching hinges → body. Path fills spine. Body = 7 nodes. 3 limbs. 4 spine, 3 hips, 5 chain hinges. **0 symmetric groups** (fully asymmetric).

**CPG:** 12 internal + 17 coupling = **29 params**

**Modules:** 15 (1 core, 12 hinges, 2 bricks)

---

## 12. Longleg (12 hinges)

**Shape:** Spine-dominated. 7 spine hinges with short legs at two branching points.

**Structure:**
- Core → left: `Hinge(90°) → Hinge(0°) → Hinge(0°) → Hinge(-90°) → Hinge(0°) → Brick` (5-hinge spine part 1) → right: `Hip(0°)`, front: `Hip(0°)`, left: `Hinge(90°) → Hinge(-90°) → Brick` (spine part 2) → right: `Hip(90°)`, left: `Hip(90°) → Hinge(0°)` (2-hinge leg)

**Hinge orientations:** Spine part 1: 90°/0°/0°/-90°/0°. Spine part 2: 90°/-90°. Legs mix 0° and 90°.

**BLF:** Part 1 end brick has 4 touching hinges → body. Part 2 end brick has 3 → body. Path fills entire spine. Body = 10 nodes. 4 limbs. **7 spine**, 4 hips, 1 chain hinge.

**CPG:** 12 internal + 34 coupling = **46 params**

**Modules:** 15 (1 core, 12 hinges, 2 bricks)

---

## 13. Penguin (12 hinges)

**Shape:** Extreme spine. 8 spine hinges with 4 small limbs. Eel-like.

**Structure:** Core → right: Brick → left: spine winds through multiple `Hinge → Hinge → Brick` segments with side branches. Spine hinges alternate 90°/-90° with some 0°.

**Hinge orientations:** Heavy 90°/-90° alternation in spine. Limb hips at 0° or 90°.

**BLF:** Three brick clusters with 3+ touching hinges → body. Path fills massive spine. Body = 14 nodes. 4 limbs. **8 spine** (highest), 4 hips. Most spine hinges of any robot.

**CPG:** 12 internal + 40 coupling = **52 params**

**Modules:** 19 (1 core, 12 hinges, 6 bricks)

---

## 14. Pentapod (10 hinges)

**Shape:** 5-legged. Long spine with single-hinge legs at three branching points.

**Structure:**
- Core → right: `Hinge(90°) → Hinge(0°) → Hinge(0°) → Hinge(-90°) → Brick` (4-hinge spine chain) → left: `Hip(0°)`, front: `Hinge(90°) → Brick` → right: `Hip(0°)`, front: `Hinge(90°) → Brick` → left: `Hip(0°)`, right: `Hip(0°)`

**Hinge orientations:** Spine 90°/0°/0°/-90°/90°/90°. All leg hips at 0°.

**BLF:** Three brick clusters with 3+ touching hinges → body. Body = 11 nodes. 4 limbs (one brick-only stub is part of body, not a limb). 6 spine, 4 hips.

**CPG:** 10 internal + 27 coupling = **37 params**

**Modules:** 15 (1 core, 10 hinges, 4 bricks)

---

## 15. Queen (9 hinges)

**Shape:** Asymmetric body with T-shaped brick cluster and 5 limbs.

**Structure:**
- Core → back: `Hinge(90°)` (stub)
- Core → right: `Hinge(90°) → Hinge(0°) → Hinge(-90°) → Brick` (3-hinge spine) → left: `Hip(0°)`, right: Brick → front: Brick (with left: `Hip(0°)`, right: `Hip(0°)`) and right: Brick → front: `Hip(90°) → Hinge(0°)` (2-hinge limb)

**Hinge orientations:** Spine 90°/0°/-90°. Stub 90°. Branch hips 0°. 2-hinge limb 90°/0°.

**BLF:** 4-brick T-cluster has 5 touching hinges → body. Path fills spine from core. Body = 8 nodes. 5 limbs. 3 spine, 5 hips, 1 chain hinge.

**CPG:** 9 internal + 19 coupling = **28 params**

**Modules:** 14 (1 core, 9 hinges, 4 bricks)

---

## 16. Squarish (6 hinges)

**Shape:** Compact, roughly square footprint. Short spine with legs and a brick tail.

**Structure:**
- Core → back: `Hinge(0°) → Brick` → front: `Hinge(0°)`, left: `Hinge(90°) → Brick → Brick` → left: `Hip(90°)`, front: `Hip(0°)`, right: `Hip(90°) → Brick → Brick → Brick` (brick tail)

**Hinge orientations:** Three at 90°, three at 0°.

**BLF:** 1-brick cluster with 3 touching hinges → body. 2-brick cluster with 4 touching → body. Path fills spine. Body = 6 nodes. 4 limbs (1 has a 3-brick tail). 2 spine, 4 hips.

**CPG:** 6 internal + 12 coupling = **18 params**

**Modules:** 13 (1 core, 6 hinges, 6 bricks)

---

## 17. Snake (8 hinges)

**Shape:** Pure linear chain. No branches.

**Structure:** Core → left: alternating `Hinge → Brick` for 8 hinges and 7 bricks, ending with a bare hinge.

**Hinge orientations:** Strictly alternating 0°/90°/0°/90°/0°/90°/0°/90°. Creates 3D sidewinding.

**BLF:** No brick cluster has >2 touching hinges (all are chain spacers with exactly 2). Body = core only. **1 single limb** with all 15 non-core modules. 0 spine, 1 hip, 7 chain hinges. 0 symmetric groups. Degenerate case — BLF cannot identify a spine because there's no branching.

**CPG:** 8 internal + 7 coupling = **15 params**

**Modules:** 16 (1 core, 8 hinges, 7 bricks)

---

## 18. Stingray (9 hinges)

**Shape:** Multi-branch spine, similar to park. 6 small limbs.

**Structure:**
- Core → back: `Hinge(90°)` (stub)
- Core → right: `Hinge(90°) → Hinge(-90°) → Brick` (spine) → branches at two brick junctions with hinges at 0° and 90°

**Hinge orientations:** Spine 90°/-90°. Stub 90°. Branch hinges alternate 0° and 90°.

**BLF:** 3-brick cluster with 5 touching hinges → body. 1-brick cluster with 3 touching → body. Path fills spine. Body = 8 nodes. 6 limbs. 3 spine, 6 hips.

**CPG:** 9 internal + 24 coupling = **33 params**

**Modules:** 15 (1 core, 9 hinges, 5 bricks)

---

## 19. Tinlicker (8 hinges)

**Shape:** Long spine (5 spine hinges) with limbs clustered at the far end.

**Structure:**
- Core → right: `Hinge(90°) → Hinge(0°) → Hinge(0°) → Hinge(-90°) → Brick` (4-hinge spine chain) → branching through bricks with 3 limb hips at 90°/0°/90°/90°

**Hinge orientations:** Spine 90°/0°/0°/-90°. Limb hips: 90°, 0°, 90°, 90°.

**BLF:** 4-brick cluster with 3 touching hinges → body. 2-brick cluster with 3 touching → body. Path fills spine. Body = 12 nodes. 3 limbs. 5 spine, 3 hips. All limbs at one end.

**CPG:** 8 internal + 17 coupling = **25 params**

**Modules:** 15 (1 core, 8 hinges, 6 bricks)

---

## 20. Turtle (13 hinges)

**Shape:** Complex asymmetric. Spine with varying limb sizes and a 4-hinge chain.

**Structure:** Core → left: Brick → branches. Spine winds through `Hinge(90°) → Hinge(-90°) → Brick` segments. One limb extends as `Hip → Hinge → Hinge → Hinge` (4-hinge chain). Heavy use of 90°/-90° pairs.

**Hinge orientations:** Spine 90°/-90°/0°. 4-hinge chain 90°/-90°/0°/0°. 2-hinge limb 90°/-90°.

**BLF:** Three brick clusters with 3+ touching hinges → body. Path fills spine. Body = 11 nodes. 5 limbs. 4 spine, 5 hips, 4 chain hinges. 1 symmetric group.

**CPG:** 13 internal + 27 coupling = **40 params**

**Modules:** 20 (1 core, 13 hinges, 6 bricks)

---

## 21. WW (10 hinges)

**Shape:** L-shaped spine with one long 3-hinge chain limb. Only 3 limbs.

**Structure:**
- Core → back: `Hinge(0°)` (stub)
- Core → right: `Hinge(90°) → Hinge(0°) → Hinge(-90°) → Brick` (3-hinge spine) → left: `Hinge(0°) → Brick` → left: `Hip(0°)`, front: Brick → right: `Hip(90°) → Brick` → left: `Hip(90°) → Hinge(0°) → Hinge(-90°)` (3-hinge chain)

**Hinge orientations:** Spine 90°/0°/-90°. Stub 0°. Long chain 90°/90°/0°/-90°.

**BLF:** 2-brick cluster with 3 touching hinges → body. Path fills spine. Body = 8 nodes. 3 limbs (fewest of any legged robot). 4 spine, 3 hips, 3 chain hinges.

**CPG:** 10 internal + 15 coupling = **25 params**

**Modules:** 15 (1 core, 10 hinges, 4 bricks)

---

## 22. Zappa (11 hinges)

**Shape:** Very long spine (6 spine hinges) with 4 limbs at the far end.

**Structure:**
- Core → back: `Hinge(0°)` (stub)
- Core → right: `Hinge(90°) → Hinge(0°) → Hinge(0°) → Hinge(-90°) → Hinge(0°) → Brick` (5-hinge spine chain) → front: `Hip(0°) → Hinge(0°)` (2-hinge limb), left: `Hinge(90°) → Brick` → left: `Hip(0°) → Brick`, front: `Hip(0°)`

**Hinge orientations:** Spine 90°/0°/0°/-90°/0°. Stub 0°. Limb hips mostly 0°, one at 90°.

**BLF:** Two brick clusters with 3+ touching hinges → body. Path fills spine. Body = 9 nodes. 4 limbs. 6 spine, 4 hips, 1 chain hinge. All limbs at one end.

**CPG:** 11 internal + 27 coupling = **38 params**

**Modules:** 15 (1 core, 11 hinges, 3 bricks)

---

## Summary Table

| # | Robot | Hinges | Bricks | Total | Spine | Hips | Chain | Limbs | Sym | CPG params | Category |
|---|-------|--------|--------|-------|-------|------|-------|-------|-----|------------|----------|
| 1 | spider | 8 | 8 | 17 | 0 | 4 | 4 | 4 | 1 | 18 | Symmetric quadruped |
| 2 | gecko | 6 | 6 | 13 | 2 | 4 | 0 | 4 | 1 | 17 | Elongated quadruped |
| 3 | babya | 8 | 7 | 16 | 2 | 4 | 2 | 4 | 1 | 21 | Asymmetric quadruped |
| 4 | babyb | 10 | 10 | 21 | 0 | 4 | 6 | 4 | 1 | 22 | Heavy quadruped |
| 5 | ant | 8 | 8 | 17 | 2 | 6 | 0 | 6 | 1 | 32 | Hexapod |
| 6 | salamander | 14 | 10 | 25 | 2 | 8 | 4 | 8 | 1 | 55 | Long-spine multi-leg |
| 7 | blokky | 5 | 9 | 15 | 0 | 2 | 3 | 2 | 0 | 9 | Compact blob |
| 8 | park | 8 | 6 | 15 | 3 | 5 | 0 | 5 | 1 | 26 | Multi-branch spine |
| 9 | garrix | 10 | 4 | 15 | 4 | 4 | 2 | 5 | 1 | 28 | Asymmetric L-shape |
| 10 | insect | 9 | 2 | 12 | 3 | 4 | 2 | 4 | 1 | 26 | Compact high-actuation |
| 11 | linkin | 12 | 2 | 15 | 4 | 3 | 5 | 3 | 0 | 29 | Chain/tentacle |
| 12 | longleg | 12 | 2 | 15 | 7 | 4 | 1 | 4 | 1 | 46 | Spine-dominated |
| 13 | penguin | 12 | 6 | 19 | 8 | 4 | 0 | 4 | 1 | 52 | Extreme spine |
| 14 | pentapod | 10 | 4 | 15 | 6 | 4 | 0 | 4 | 1 | 37 | 5-legged spine |
| 15 | queen | 9 | 4 | 14 | 3 | 5 | 1 | 5 | 1 | 28 | Asymmetric multi-limb |
| 16 | squarish | 6 | 6 | 13 | 2 | 4 | 0 | 4 | 1 | 18 | Compact quadruped |
| 17 | snake | 8 | 7 | 16 | 0 | 1 | 7 | 1 | 0 | 15 | Linear chain |
| 18 | stingray | 9 | 5 | 15 | 3 | 6 | 0 | 6 | 1 | 33 | Multi-branch spine |
| 19 | tinlicker | 8 | 6 | 15 | 5 | 3 | 0 | 3 | 1 | 25 | End-branching spine |
| 20 | turtle | 13 | 6 | 20 | 4 | 5 | 4 | 5 | 1 | 40 | Complex asymmetric |
| 21 | ww | 10 | 4 | 15 | 4 | 3 | 3 | 3 | 1 | 25 | L-shaped spine |
| 22 | zappa | 11 | 3 | 15 | 6 | 4 | 1 | 4 | 1 | 38 | End-branching spine |

## Morphology Categories

**Category A -- Symmetric quadrupeds** (clean BLF cases):
- spider (4 identical 2-hinge legs, no spine)
- babyb (3 long + 1 short leg, no spine)
- squarish (4 legs, compact, short spine)

**Category B -- Elongated with spine and legs** (tests spine + limb separation):
- gecko (2 spine, 4 single-hinge legs)
- ant (2 spine, 6 legs -- hexapod)
- babya (asymmetric gecko variant)

**Category C -- Spine-dominated** (most actuation in spine):
- longleg (7 spine, 4 tiny legs)
- penguin (8 spine, 4 tiny legs)
- pentapod (6 spine, 4 legs)
- tinlicker (5 spine, legs at one end)
- zappa (6 spine, legs at one end)

**Category D -- Complex/asymmetric** (challenges BLF):
- salamander (14 hinges, long body, many stubs)
- turtle (13 hinges, complex branching)
- linkin (12 hinges, fully asymmetric, 0 sym groups)
- garrix (L-shaped, long hinge chains)

**Category E -- Degenerate/edge cases**:
- snake (linear chain, no branching -- BLF has no spine to detect)
- blokky (5 hinges, mostly passive bricks)

**Category F -- Multi-branch** (many small limbs):
- park (5 limbs, 3 spine)
- stingray (6 limbs, 3 spine)
- queen (5 limbs, T-shaped body)
