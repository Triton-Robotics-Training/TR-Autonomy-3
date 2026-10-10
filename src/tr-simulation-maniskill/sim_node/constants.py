MAX_TRANSLATION_VEL_M_S = 1.0
MAX_ANGULAR_VEL_RADS_S = 4.5

TELEOP_TRANSLATION_VEL_M_S = 0.6
TELEOP_ANGULAR_VEL_RADS_S = 1.0

# The secondary robot (the aiming target) starts out spinning counterclockwise
# at this rate, the same as pressing "4" in keyboard_controls, so there is
# something to track as soon as the CV stack comes up. Both sim_node and
# keyboard_controls start from it; keyboard_controls publishes continuously, so
# if it started from 0 it would stop the spin the moment it was launched.
TARGET_START_ANGULAR_VEL_RADS_S = MAX_ANGULAR_VEL_RADS_S * 0.25
