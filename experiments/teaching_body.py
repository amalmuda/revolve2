"""Teaching body for Chapter 3 figures.

Small, asymmetric V1 body that exercises every BLF classification rule and
every coupling topology rule. Used only for figures; never entered into
any evolution run.

Layout (12 modules):

    core
      front:  BRICK_CF  (core-attached brick -> body via Rule 1 / Rule 3)
        front:  HINGE_SPINE  (spine hinge on path between body regions -> Rule 3 + Rule 5 spine)
          attachment: BRICK_JA  (cluster piece 1)
            front: BRICK_JB  (cluster piece 2 -> cluster {JA, JB} is body by Rule 2)
              front:  HINGE_C -> HINGE_F -> BRICK_FOOT  (multi-hinge limb ending in brick-foot)
              left:   HINGE_D -> HINGE_G                 (limb ending in a hinge -> hinge-foot)
              right:  HINGE_E -> BRICK_FOOT              (single-hinge limb ending in brick-foot)

Hinges touching cluster {JA, JB}: spine, HINGE_C, HINGE_D, HINGE_E = 4 > 2
-> Rule 2 fires.

Path from core to {JA, JB}: core -> BRICK_CF -> HINGE_SPINE -> JA.
-> Rule 3 marks BRICK_CF and HINGE_SPINE as body.
-> Rule 5 then types HINGE_SPINE as spine.
"""
import numpy as np
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1


def teaching_body_v1() -> BodyV1:
    body = BodyV1()

    # Core-attached brick (on path to cluster)
    body.core_v1.front = BrickV1(0.0)

    # Spine hinge (on path between core-body and cluster-body)
    body.core_v1.front.front = ActiveHingeV1(np.pi / 2.0)

    # Cluster piece 1
    body.core_v1.front.front.attachment = BrickV1(-np.pi / 2.0)

    # Cluster piece 2 (extends cluster)
    body.core_v1.front.front.attachment.front = BrickV1(0.0)

    # --- Three limbs branching from BRICK_JB ---

    # Limb 1: multi-hinge (hip + knee) ending in a foot-brick
    body.core_v1.front.front.attachment.front.front = ActiveHingeV1(0.0)           # HINGE_C (hip)
    body.core_v1.front.front.attachment.front.front.attachment = ActiveHingeV1(0.0)  # HINGE_F (knee)
    body.core_v1.front.front.attachment.front.front.attachment.attachment = BrickV1(0.0)  # foot-brick

    # Limb 2: two hinges, nothing after the second -> the last hinge itself is the foot
    body.core_v1.front.front.attachment.front.left = ActiveHingeV1(0.0)            # HINGE_D (hip)
    body.core_v1.front.front.attachment.front.left.attachment = ActiveHingeV1(0.0) # HINGE_G (knee + foot)

    # Limb 3: single hinge ending in a foot-brick
    body.core_v1.front.front.attachment.front.right = ActiveHingeV1(np.pi / 2.0)   # HINGE_E (hip)
    body.core_v1.front.front.attachment.front.right.attachment = BrickV1(0.0)      # foot-brick

    return body
