
from roombasim.pittras import PittRASDrone, render
from roombasim.pittras.ai import *
from roombasim.pittras.state import *
from roombasim.pittras.task import *

import numpy as np

#
# IMPLEMENTATION SPECIFIC CONSTANTS
#
AGENT = PittRASDrone

DRONE_MAX_VERTICAL_VELOCITY = 1.0
DRONE_MAX_HORIZ_ACCEL = 3.0
DRONE_MAX_HORIZ_VELOCITY = 1.0

CONTROLLER = WaypointDemoController

TASKS = {
    'HoldPositionTask': HoldPositionTask,
    'XYZTranslationTask': XYZTranslationTask,
    'TakeoffTask': TakeoffTask,
    'GoToRoombaTask': GoToRoombaTask,
    'TrackRoombaTask': TrackRoombaTask,
    'HitRoombaTask': HitRoombaTask,
    'LandTask': LandTask,
    'BlockRoombaTask': BlockRoombaTask,
    'VelocityTask': VelocityTask
}

STATES = {
    'DroneState': DroneState,
    'RoombaState': RoombaState
}

RENDER_AGENT = render.render_pittrasdrone

#
# PITTRAS DRONE CONFIGURATION
#

# the width of the square bumper base in meters
PITTRAS_DRONE_BASE_WIDTH = 0.57

# the distance from the center to a corner in meters
PITTRAS_DRONE_BASE_DIAGONAL = (PITTRAS_DRONE_BASE_WIDTH / 2) * np.sqrt(2)

# the radius of the roomba bumper pad im meters
PITTRAS_DRONE_PAD_RADIUS = 0.175

# altitude of the drone that will cause contact
# with the roombas
PITTRAS_DRONE_PAD_ACTIVIATION_HEIGHT = 0.03

# radius of outer edge of prop guards in meters
PITTRAS_DRONE_PROP_RADIUS = 0.155

# velocity with which to take off with
PITTRAS_TAKEOFF_VELOCITY = 0.3

# height at which to consider takeoff finished in meters
# should be higher than min_maneuver_height
PITTRAS_TAKEOFF_COMPLETE_HEIGHT = 0.7

# delay between arming and taking off
PITTRAS_DELAY_BEFORE_TAKEOFF = 2.0

# altitude when we transition from acro to angle mode
PITTRAS_TAKEOFF_ANGLE_MODE_HEIGHT = 0.2

# timeout for transforms
PITTRAS_TAKEOFF_TRANSFORM_TIMEOUT = 0.2

# velocity with which to land with
PITTRAS_LAND_VELOCITY = -0.2

# distance from the target that the XYZTranslationTask
# will deem "close enough" in meters
PITTRAS_XYZ_TRANSLATION_ACCURACY = 0.2

# PID constants for xy controller
# [Kp,Kd,Ki]
PITTRAS_PID_XY = np.array([0.5, 1.1, 0])

# PID constants for z controller
# [Kp,Kd,Ki]
PITTRAS_PID_Z = np.array([0.9, 0, 0])

# PID constants for yaw controller
# [Kp,Kd,Ki]
PITTRAS_PID_YAW = np.array([0.5, 0, 0])

# hover height while tracking roomba in meters
PITTRAS_TRACK_ROOMBA_HEIGHT = 1

# minimum distance to allow hit roomba task to run in meters
PITTRAS_HIT_ROOMBA_MAX_START_DIST = 0.5

# descent velocity for hit roomba task
PITTRAS_HIT_DESCENT_VELOCITY = -0.2

PITTRAS_BLOCK_DESCENT_VEL = -0.3

# Tolerance for distance comparasion in the HoldPositionTask
PITTRAS_HOLD_POSITION_TOLERANCE = 0.2

# Tolerance for speed comparasion in the VelocityTask
PITTRAS_VELOCITY_TOLERANCE = 0.1
