# FILE LOGGING
# TODO FILL WITH AN ABSOLUTE PATH
EXP_LOG_PATH = '~/KvaserCar_outdoor_test/tactical_log'


# CRITICAL REGION POINTS
#  critical region
#    (P4)-----(P1)
#     |         |
#     |         |
#    (P3)-----(P2)
#
CR_POINT_1 = [0.5, 0.5]
CR_POINT_2 = [0.5, -0.5]
CR_POINT_3 = [-0.5, -0.5]
CR_POINT_4 = [-0.5, 0.5]

# ADVERSARY VEHICLE PATH POINTS
ADV_PATH_START = [0.0, -4.0]
ADV_PATH_END = [0.0, 2.0]

# EGO VEHICLE PATH POINTS
EGO_PATH_START = [4.0, 0.0]
EGO_PATH_END = [-2.0, 0.0]

# ADVERSARY PARAMETERS
ADV_MAX_SPEED = 1.25 #[m/s]
ADV_MAX_ACC = 1.05 #[m/s]
ADV_LENGTH = 0.720 #[m]
ADV_WIDTH = 0.515 #[m]

# EGO PARAMETERS
EGO_REFERENCE_SPEED = 1.5 #[m/s]
EGO_MAX_ACC = 1.35 # [m/s]
EGO_MAX_DEC = 1.25 # [m/s]
EGO_LENGTH = 0.720 #[m]
EGO_WIDTH = 0.515 #[m]
