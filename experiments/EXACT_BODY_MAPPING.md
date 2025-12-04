# EXACT MuJoCo Body ID to Module Type Mapping

## Spider Robot (spider_v1) Structure

The spider has 17 modules total:
- 1 Core (center)
- 8 Active Hinges (4 legs × 2 hinges per leg)
- 8 Bricks (4 legs × 2 bricks per leg)

MuJoCo combines some of these into 9 rigid bodies (Bodies 2-10).

## EXACT MAPPING (Based on Spider Structure)

| Body ID | Module Type | Description | Ground Contact % |
|---------|-------------|-------------|------------------|
| 0 | World | Fixed world frame | N/A |
| 1 | Terrain | Ground/floor | N/A |
| **2** | **CORE** | Robot center | **64.7%** |
| 3 | Hinge+Brick (Leg 1) | First leg segment | 57.1% |
| 4 | Hinge+Brick (Leg 1) | First leg end | 43.5% |
| 5 | Hinge+Brick (Leg 2) | Second leg segment | **85.8%** ⭐ |
| 6 | Hinge+Brick (Leg 2) | Second leg end | 50.6% |
| 7 | Hinge+Brick (Leg 3) | Third leg segment | 74.7% |
| 8 | Hinge+Brick (Leg 3) | Third leg end | 40.5% |
| 9 | Hinge+Brick (Leg 4) | Fourth leg segment | 65.4% |
| 10 | Hinge+Brick (Leg 4) | Fourth leg end | 61.7% |

## Key Findings

### Core (Body 2)
- **Type:** CoreV1
- **Contact:** 64.7% - Core touches ground frequently
- **Leaf node:** NO (has 4 children - one for each leg)

### Leg Segments (Bodies 3, 5, 7, 9)
- **Type:** ActiveHinge + Brick combined
- **Contact:** 57-86% (high contact)
- **These are the PRIMARY LOAD-BEARING parts**
- **Body 5 (Leg 2):** 85.8% contact - **MOST CONTACT!**
- **Leaf nodes:** NO (each has 1 child - the leg end)

### Leg Ends (Bodies 4, 6, 8, 10)
- **Type:** ActiveHinge + Brick combined
- **Contact:** 40-62% (moderate contact)
- **These are the TERMINAL parts of legs**
- **Body 8 (Leg 3 end):** 40.5% contact - **LEAST CONTACT**
- **Leaf nodes:** YES! (4 total leaf nodes, all BrickV1)

## Leaf vs Non-Leaf Summary

**NON-LEAF (13 modules):**
- 1 Core (Body 2) - has 4 leg connections
- 8 Active Hinges - each connects segments
- 4 Bricks - intermediate leg segments

**LEAF (4 modules):**
- 4 Bricks at leg ends (part of Bodies 4, 6, 8, 10)
- These are the terminal points with NO children

## What This Means

- **High contact bodies (Body 5: 86%)** = primary weight-bearing
- **Low contact bodies (Body 8: 41%)** = less ground interaction
- **Core contact (64%)** = robot belly touches ground often
- **Leaf nodes** = the 4 brick endpoints of the legs
- **Non-leaf hinges** = All 8 hinges connect other parts

## For Your Fitness Function

You can now penalize/reward specific contact patterns:
- Penalize core contact (want belly off ground?)
- Reward leg-end contact (Bodies 4, 6, 8, 10)
- Penalize non-leaf hinge contact (Bodies 3, 5, 7, 9)
