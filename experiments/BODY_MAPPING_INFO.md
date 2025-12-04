# Robot Body ID Mapping

## Spider Robot (spider_v1)

### Module Composition
- **Total modules:** 17
  - 1 Core (center of robot)
  - 8 Active Hinges (joints that move)
  - 8 Bricks (rigid body segments)

### MuJoCo Body IDs

The robot has 17 modules but only 11 MuJoCo bodies because some modules are combined into single rigid bodies.

| Body ID | Name | Type | Description |
|---------|------|------|-------------|
| 0 | world | World | Fixed world reference frame |
| 1 | mbs0/ | Terrain | Flat ground/terrain |
| 2 | mbs1/ | **Core** | Robot's core/center (root body) |
| 3 | mbs1/mbs1_link0 | **Leg 1 part** | First leg segment |
| 4 | mbs1/mbs1_link0_link1 | **Leg 1 part** | First leg continued |
| 5 | mbs1/mbs1_link1 | **Leg 2 part** | Second leg segment |
| 6 | mbs1/mbs1_link1_link1 | **Leg 2 part** | Second leg continued |
| 7 | mbs1/mbs1_link2 | **Leg 3 part** | Third leg segment |
| 8 | mbs1/mbs1_link2_link1 | **Leg 3 part** | Third leg continued |
| 9 | mbs1/mbs1_link3 | **Leg 4 part** | Fourth leg segment |
| 10 | mbs1/mbs1_link3_link1 | **Leg 4 part** | Fourth leg continued |

## Contact Detection Results

From the working simulation (test_contacts_viz.py):

```
Fitness: 0.6312 m
Total contacts: 163,202
Bodies in ground contact: 9 (all except world and terrain)

Per-body contact frequency:
  Body 2 (Core): 64.7% - Center body touches ground often
  Body 5 (Leg 2): 85.8% - Most ground contact
  Body 7 (Leg 3): 74.7% - High ground contact
  Body 9 (Leg 4): 65.4% - High ground contact
  Body 3 (Leg 1): 57.1% - Moderate contact
  Body 6 (Leg 2 cont): 50.6% - Half the time
  Body 4 (Leg 1 cont): 43.5% - Less contact
  Body 8 (Leg 3 cont): 40.5% - Least contact
  Body 10 (Leg 4 cont): 61.7% - Moderate-high contact
```

## Notes

- **Bodies 2-10** are all robot parts (1 core + 8 leg segments)
- **Body 1** is the terrain
- **Body 0** is the world frame
- Each "leg" consists of: Hinge → Brick → Hinge → Brick (but may be combined in MuJoCo)
- High contact percentage = that body part touches ground frequently during movement
- Bodies with 85%+ contact are primary load-bearing/locomotion parts

## Module Types

- **Core:** The central body of the robot
- **ActiveHinge:** Motorized joints that rotate (controlled by CPG brain)
- **Brick:** Rigid structural segments connecting hinges
- **Terrain:** The ground surface (static)
