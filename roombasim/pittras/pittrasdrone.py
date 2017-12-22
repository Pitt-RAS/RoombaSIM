'''
pittrasdrone.py

Drone implementation for the Univeristy of Pittsburgh's
Robotic Automation Society.
'''
import numpy as np

from roombasim.agent.drone import Drone
import roombasim.config as cfg
from roombasim import geometry

class PittRASDrone(Drone):

    def update(self, delta, elapsed):
        super(PittRASDrone, self).update(delta, elapsed)

    def is_touching_roomba_top(self, rba):
        '''
        PittRAS drone pad has a diameter of 35cm.
        '''
        dist2 = pow((self.xy_pos[0] - rba.pos[0]), 2) + pow((self.xy_pos[1] - rba.pos[1]), 2)
        dist = np.sqrt(dist2)

        return self.z_pos < cfg.PITTRAS_DRONE_PAD_ACTIVIATION_HEIGHT and dist < cfg.PITTRAS_DRONE_PAD_RADIUS

    def is_blocking_roomba(self, rba):
        '''
        PittRAS drone has a square base of width: 57cm
        '''
        hit_bumper = geometry.circle_intersects_square(rba.pos, cfg.ROOMBA_RADIUS, self.xy_pos, self.yaw, cfg.PITTRAS_DRONE_BASE_WIDTH)
        
        return hit_bumper and self.z_pos < cfg.PITTRAS_DRONE_PAD_ACTIVIATION_HEIGHT
