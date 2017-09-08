'''
mission.py

Contains a Mission class that can be used to
setup initial configurations and perform game-wide
update steps.

Additionally, this class is responsible for performing
collision detection and notifying child roombas via
the collision dict.
'''

import numpy as np

import config as cfg
import roomba

class Mission(object):
    '''
    A class to represent a game round.

    Contains methods to initialize a round and update
    methods that can be used to progress through time.
    '''

    def __init__(self):
        self.roombas = []
        self.quad = None

    def setup(self):
        '''
        Spawns roombas and positions them as follows:

        - 10 target roombas evenly spaced around a 1m circle
        centered at the origin. All roombas initially face
        outwards.

        - 4 obstacle roombas evenly spaced around a 4m circle
        centered at the origin. Roombas move clockwise around
        the circle.
        '''

        # spawn target roombas
        for i in range(cfg.MISSION_NUM_TARGETS):
            theta = (cfg.TAU * i) / cfg.MISSION_NUM_TARGETS

            target_roomba = roomba.TargetRoomba(
                [np.cos(theta) * cfg.MISSION_TARGET_SPAWN_RADIUS + 10, np.sin(theta) * cfg.MISSION_TARGET_SPAWN_RADIUS + 10],
                theta
            )

            target_roomba.start()

            self.roombas.append(target_roomba)

        # spawn obstacle roombas
        for i in range(cfg.MISSION_NUM_OBSTACLES):
            theta = (cfg.TAU * i) / cfg.MISSION_NUM_OBSTACLES

            obstacle_roomba = roomba.ObstacleRoomba(
                [np.cos(theta) * cfg.MISSION_OBSTACLE_SPAWN_RADIUS + 10, np.sin(theta) * cfg.MISSION_OBSTACLE_SPAWN_RADIUS + 10],
                theta - (cfg.PI / 2)
            )

            obstacle_roomba.start()

            self.roombas.append(obstacle_roomba)
        
    def update(self, delta, elapsed):
        '''
        Perform an update step.

        This will update all child roombas and perform collision
        detection in O(n^2) time. At the current stage, this is
        "good enough" for realtime display but for high-speed
        simulation in a neural network training phase it may
        make sense to improve the efficiency of certain aspects
        of the code.
        '''
        for i in range(len(self.roombas)):
            rba = self.roombas[i]
            rba.update(delta, elapsed)

            for j in range(len(self.roombas)):
                if i == j:
                    continue

                if Mission._check_roomba_collision(rba, self.roombas[j]):
                    if Mission._check_roomba_is_facing(rba, self.roombas[j]):
                        rba.collisions['front'] = True


    @staticmethod
    def _check_roomba_collision(ra, rb):
        '''
        Returns true if two roombas are touching.

        The calculation is done purely by determining the
        euclidean distance and comparing that to the radius
        of each roomba.
        '''
        return pow(ra.pos[0] - rb.pos[0], 2) + pow(ra.pos[1] - rb.pos[1], 2) < (4 * cfg.ROOMBA_RADIUS * cfg.ROOMBA_RADIUS)

    @staticmethod
    def _check_roomba_is_facing(ra, rb):
        '''
        Returns true if roomba ra is facing roomba rb.

        This works by determining the angle of the vector
        ra -> rb relative to the +x axisd and checking if the 
        heading of ra is within pi/2 radians of that result.
        '''
        ang = np.arctan2(rb.pos[1] - ra.pos[1], rb.pos[0] - ra.pos[0])
        return Mission._compare_angle(ra.heading, ang) < cfg.PI / 2
    
    @staticmethod
    def _compare_angle(a, b):
        '''
        Returns the smallest absolute difference between the 
        two angles in radians.

        a, b - two angles to compare in radians
        '''
        return abs((((a - b) + cfg.PI) % cfg.TAU) - cfg.PI)
    