
from pyglet.gl import *
import numpy as np

import roombasim.config as cfg
from roombasim.graphics.display import Display
from roombasim import geometry

def render_pittrasdrone(drone):
    glEnable(GL_BLEND)
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT,GL_NICEST)

    # altitude indicator
    # alpha is 1 when landed and 0 when >= 2 meters
    alpha = max(min(((-0.5 * drone.altitude) + 1), 1), 0)
    glColor4f(0.5,0.5,0.5,alpha)

    scale = ((1 - alpha) * 3) + 1
    Display._draw_hollow_square(drone.pos, drone.heading, cfg.PITTRAS_DRONE_BASE_DIAGONAL * scale)

    # draw bumpers
    if drone.altitude <= cfg.PITTRAS_DRONE_PAD_ACTIVIATION_HEIGHT:
        glColor3f(1,0.5,0.5)
    else:
        glColor3f(1,1,1)
    Display._draw_hollow_square(drone.pos, drone.heading, cfg.PITTRAS_DRONE_BASE_DIAGONAL)

    # draw prop guards
    glColor3f(0.8,0.8,0.5)
    for c in geometry.get_square_corners(drone.pos, drone.heading, cfg.PITTRAS_DRONE_BASE_WIDTH):
        Display._draw_hollow_circle(c, cfg.PITTRAS_DRONE_PROP_RADIUS)

    # Direction indicator
    glColor3f(1,1,1)
    glBegin(GL_LINES)

    glVertex2f(drone.pos[0], drone.pos[1])
    glVertex2f(
        np.cos(drone.heading) * (cfg.PITTRAS_DRONE_BASE_WIDTH / 2) + drone.pos[0], 
        np.sin(drone.heading) * (cfg.PITTRAS_DRONE_BASE_WIDTH / 2) + drone.pos[1]
    )

    glEnd()