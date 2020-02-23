#
#   gravity.py
#
#   Gravity Torque Computation
#
import math
import random

#
#   Gravity Torque
#
def gravity(pos):
    # Set the torque offsets - these should really be cleared by
    # reseting the actuator force sensors...
    off0 = 0.0
    off1 = 0.0
    off2 = 0.0

    # Compute from the tip inward
    shoulder = pos[1]
    elbow = (pos[2] -0.1) * 2
    wrist = (pos[3]-0.09) * 3
    
    # print("perceived:", shoulder, elbow, wrist, "into sin:", shoulder,
    #     shoulder + elbow - math.pi / 2,
    #     shoulder + elbow - wrist - math.pi)
    grav2 = 0.12 * math.sin(shoulder + elbow - wrist - math.pi)
    grav1 = -(5.16 ) * math.sin(shoulder + elbow - math.pi / 2) - grav2 / 3 \
        -0.7 * math.cos(shoulder + elbow + 3.14 - math.pi / 2)
    grav0 = -14.4 * math.sin(shoulder) + grav1 / 2 \
        - 0.5 * math.cos(shoulder)

    # Return the gravity vector
    return [0, grav0+off0, grav1+off1, grav2+off2, 0, 0]
