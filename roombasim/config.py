'''
config.py

Contains a bunch of modifiable constants organized by
module type.
'''
import numpy as np

#
# MATH CONSTANTS
#

# please don't change these ;p
PI = np.pi
TAU = np.pi * 2

#
# ROOMBA CONFIGURATION
#

# Speed of the roomba moving forwards
ROOMBA_LINEAR_SPEED = 0.33 # m/s

# Turning speed of the roomba
ROOMBA_ANGULAR_SPEED = 1.279 # rad/s

# Time until a full reverse (milliseconds)
ROOMBA_REVERSE_PERIOD = 20000

# Time until random heading noise is applied (milliseconds)
ROOMBA_HEADING_NOISE_PERIOD = 5000

# Maximum heading noise (applied in either direction) in radians
ROOMBA_HEADING_NOISE_MAX = 20 * (np.pi / 180)

# Python doesn't have enums...
ROOMBA_STATE_IDLE = 0
ROOMBA_STATE_FORWARD = 1
ROOMBA_STATE_TURNING = 2

# Roomba's radius in meters
ROOMBA_RADIUS = 0.35 / 2

#
# MISSION CONFIGURATION
#

# number of target roombas to spawn
MISSION_NUM_TARGETS = 10

# radius to spawn target roombas (centered at origin) in meters
MISSION_TARGET_SPAWN_RADIUS = 1

# number of obstacle roombas to spawn
MISSION_NUM_OBSTACLES = 4

# radius to spawn obstacle roombas in meters
MISSION_OBSTACLE_SPAWN_RADIUS = 4

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

#
# GRAPHICS CONFIGURATION
#

# how many vertices to use to draw circles
# (note: hopefully someone can implement fragment shaders
# and this will become irrelevant)
GRAPHICS_CIRCLE_VERTICES = 100
