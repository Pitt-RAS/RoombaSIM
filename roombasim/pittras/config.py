
import pittrasdrone
import render

import numpy as np

#
# IMPLEMENTATION SPECIFIC CONSTANTS
#
AGENT = pittrasdrone.PittRASDrone

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
