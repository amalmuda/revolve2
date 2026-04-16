# Revolve2 V1 Module Physical Specifications

All values from source code. Dimensions are total extents (not half-extents). All modules are V1 variants.

## Core V1

| Property | Value |
|---|---|
| Mass | 0.250 kg (250 g) |
| Dimensions (X x Y x Z) | 89.0 x 89.0 x 60.3 mm |
| Attachment slots | 4 (front, right, back, left) |
| Child offset | 44.5 mm |
| Color | Red (255, 50, 50) |

## Brick V1

| Property | Value |
|---|---|
| Mass | 0.030 kg (30 g) |
| Dimensions (X x Y x Z) | 62.9 x 62.9 x 60.3 mm |
| Attachment slots | 3 (front, left, right) |
| Child offset | 31.4 mm |
| Color | Blue (50, 50, 255) |

## Active Hinge V1

| Property | Value |
|---|---|
| **Total mass** | **0.089 kg (89 g)** |
| Frame mass | 0.011 kg |
| Frame dimensions | 18.0 x 53.0 x 16.6 mm |
| Servo 1 mass | 0.058 kg |
| Servo 1 dimensions | 58.3 x 51.2 x 20.0 mm |
| Servo 2 mass | 0.020 kg |
| Servo 2 dimensions | 2.0 x 53.0 x 53.0 mm |
| Attachment slots | 1 (attachment) |
| Color | White (255, 255, 255) |

### Joint properties

| Property | Value |
|---|---|
| Joint range | +/-1.047 rad (+/-60 deg) |
| Max torque (effort) | 0.948 Nm |
| Max angular velocity | 6.339 rad/s |
| PID proportional (P) | 5.0 |
| PID derivative (D) | 0.05 |
| Armature | 0.002 kg m^2 |
| Static friction | 1.0 |
| Dynamic friction | 1.0 |

### Structural offsets

| Property | Value |
|---|---|
| Frame offset | 45.25 mm |
| Joint offset | 11.9 mm |
| Servo offset | 29.9 mm |
| Child offset | 31.1 mm |

## Notes

- Motor specs derived from servo at 5.0V: torque = 9.6667 kgf cm x 9.807/100, velocity = 1/0.1652 x 60/360 x 2pi.
- All values are fixed defaults in V1 module definitions. Only the rotation angle parameter is configurable at construction time.
- The frame is the stationary part (stays with the parent rigid body in MuJoCo). Servo 1 and servo 2 form the moving part (a new rigid body connected by a hinge joint).
- The hinge joint axis is along the local Y-axis: Vector3([0, 1, 0]).
- MuJoCo uses position control via the PID gains to track the target angle set by the CPG controller.

## Simulation parameters

| Property | Value |
|---|---|
| Physics timestep | 0.001 s (1 kHz) |
| Control frequency | 20 Hz (0.05 s control step) |
| Simulation time | 30 s |
| Terrain | Flat plane, 20 x 20 m, friction 1.0 |
| Gravity | -9.81 m/s^2 (MuJoCo default) |

## Initial conditions

| Property | Value |
|---|---|
| Spawn position | Origin (0, 0, z_auto) with AABB ground contact |
| Initial orientation | Identity quaternion (no rotation) |
| Initial joint angles | 0 rad (T-pose) |
| CPG initial state | +/-sqrt(2)/2 = +/-0.70711 (alternating primary/antagonist) |
| Settling time | ~50 ms passive under gravity before first control at t=0.05 s |
| Randomization | None in simulation; only CMA-ES seed |
