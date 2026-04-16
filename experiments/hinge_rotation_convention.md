# Active Hinge Rotation Convention in Revolve2

## How rotation is applied

The `rotation` parameter passed to `ActiveHingeV1(rotation)` is a rotation around the module's local X-axis, applied via `Quaternion.from_eulers([rotation, 0, 0])`. The MuJoCo hinge joint axis is hard-coded as local Y: `Vector3([0, 1, 0])`. Rotating the module frame changes which world-space direction the local Y axis points, and therefore which plane the child swings in.

## The two standard values

| Rotation | Frame rotation | Joint axis becomes | Child swings in | Description |
|---|---|---|---|---|
| 0 rad | Identity (no rotation) | Parent's local Y (horizontal) | Vertical plane (up/down) | Vertical bend |
| pi/2 rad (90 deg) | 90 deg around local X | Up (vertical) | Horizontal plane (forward/back) | Horizontal swing |

- **0 deg hinge**: joint axis stays horizontal, child swings up and down. Used for knee/ankle joints that lift and lower the leg.
- **90 deg hinge**: joint axis points up, child swings forward and backward (or left and right, depending on attachment slot). Used for hip joints that move legs in the walking direction.
- **-90 deg hinge**: same plane as 90 deg but reversed initial phase.

## Attachment slot interaction

The actual world-space swing direction depends on both the hinge rotation AND which attachment slot on the parent the hinge is connected to. The core's four slots each rotate the child frame around Z:

| Core slot | Z rotation | Effect on a 90-deg hip |
|---|---|---|
| Front | 0 deg | Leg swings left-right |
| Back | 180 deg | Leg swings left-right |
| Left | 90 deg | Leg swings forward-backward |
| Right | 270 deg | Leg swings forward-backward |

In all cases the swing stays in the horizontal plane (because the joint axis points up). The attachment slot determines the swing direction within that plane.

For a 0-deg hinge on any slot, the swing is always in the vertical plane (up-down), regardless of which slot it is attached to.

## Spider example

Spider legs: `Core -> Hip(90 deg) -> Brick -> Knee(0 deg) -> Brick`

- **Hips** (90 deg): joint axis vertical, legs swing in horizontal plane. Front/back hips swing left-right, left/right hips swing forward-backward. All horizontal.
- **Knees** (0 deg): joint axis horizontal, lower leg swings up-down in vertical plane. This lifts and lowers the foot.

This is biologically intuitive: hips rotate legs to walk, knees bend legs to lift feet.

## Gecko example

Gecko spine: `Core -> Spine1(90 deg) -> Brick -> Spine2(90 deg) -> Brick`
Gecko legs: `Core.left/right -> Hip(0 deg) -> Brick`

- **Spine hinges** (90 deg): horizontal undulation, the body sways side-to-side.
- **Leg hips** (0 deg): vertical paddle, legs push against the ground up and down.

## Code references

- Rotation to quaternion: `_active_hinge.py` line 110-112: `Quaternion.from_eulers([rotation, 0, 0])`
- Joint axis: `_active_hinge_builder.py` line 113: `axis=Vector3([0.0, 1.0, 0.0])`
- Core attachment orientations: `_core.py` lines 43-60 (front=0, right=3pi/2, back=pi, left=pi/2 around Z)
