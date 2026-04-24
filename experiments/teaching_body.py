"""Teaching body for Chapter 3 figures.

Small, asymmetric V1 body that exercises every BLF classification rule and
every coupling topology rule. Used only for figures; never entered into
any evolution run.

Layout (16 modules):

    core
      left:   HIP_SIDE -> FOOT_BRICK                     (limb 4: single-hinge limb)
      front:  BRICK_CF  (core-attached brick)
        front:  HINGE_SPINE_1                            (spine hinge 1)
          attachment: BRICK_SPINE  (intermediate spine brick, on path)
            front: HINGE_SPINE_2                          (spine hinge 2)
              attachment: BRICK_JA   (cluster piece 1)
                front: BRICK_JB       (cluster piece 2)
                  front: HIP_C -> KNEE_F -> FOOT_BRICK   (limb 1: multi-hinge, foot brick)
                  left:  HIP_D -> KNEE_G                  (limb 2: ends in a hinge-foot)
                  right: HIP_E -> FOOT_BRICK              (limb 3: single-hinge, foot brick)

Hinges touching cluster {JA, JB}: spine_2, HIP_C, HIP_D, HIP_E = 4 > 2
-> Rule 2 fires.

Path core -> cluster runs: core -> BRICK_CF -> SPINE_1 -> BRICK_SPINE
-> SPINE_2 -> JA. Rule 3 marks BRICK_CF, SPINE_1, BRICK_SPINE, SPINE_2
as body. Rule 5 types the two spine hinges as SPINE.

4 limbs branch off the body. HIP_SIDE is a single-hinge limb off the
core's left face; HIP_C, HIP_D, HIP_E branch off the cluster.
"""
import numpy as np
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1


def teaching_body_v1() -> BodyV1:
    body = BodyV1()

    # --- Limb 4: single-hinge limb off the core's left side ---
    body.core_v1.left = ActiveHingeV1(0.0)                            # HIP_SIDE
    body.core_v1.left.attachment = BrickV1(0.0)                       # foot brick

    # --- Dead-end brick on the core's back face ---
    body.core_v1.back = BrickV1(0.0)                                  # BRICK_BACK

    # --- Spine chain from core's front to the cluster ---
    body.core_v1.front = BrickV1(0.0)                                 # BRICK_CF (core-attached)
    # Second brick hanging off BRICK_CF's right side
    body.core_v1.front.right = BrickV1(0.0)                           # BRICK_CF_RIGHT
    body.core_v1.front.front = ActiveHingeV1(np.pi / 2.0)             # SPINE_1 (bend-H)
    body.core_v1.front.front.attachment = BrickV1(-np.pi / 2.0)       # BRICK_SPINE (intermediate)
    body.core_v1.front.front.attachment.front = ActiveHingeV1(np.pi / 2.0)  # SPINE_2 (bend-H)
    body.core_v1.front.front.attachment.front.attachment = BrickV1(-np.pi / 2.0)  # BRICK_JA

    # Cluster: second brick extending from JA
    body.core_v1.front.front.attachment.front.attachment.front = BrickV1(0.0)  # BRICK_JB

    # Shorthand for cluster junction
    jb = body.core_v1.front.front.attachment.front.attachment.front

    # --- Limb 1: multi-hinge ending in a foot-brick ---
    jb.front = ActiveHingeV1(0.0)                                     # HIP_C
    jb.front.attachment = ActiveHingeV1(0.0)                          # KNEE_F
    jb.front.attachment.attachment = BrickV1(0.0)                     # foot brick
    # Extra brick on the farthest foot brick's right side
    jb.front.attachment.attachment.right = BrickV1(0.0)               # FOOT_SIDE_BRICK

    # --- Limb 2: two hinges, nothing after second -> last hinge is the foot ---
    jb.left = ActiveHingeV1(0.0)                                      # HIP_D
    jb.left.attachment = ActiveHingeV1(0.0)                           # KNEE_G (foot)

    # --- Limb 3: single hinge ending in a foot-brick ---
    jb.right = ActiveHingeV1(np.pi / 2.0)                             # HIP_E
    jb.right.attachment = BrickV1(0.0)                                # foot brick

    return body
