'''
roomba.py

Contains two classes to represent target and obstacle roombas.

Behavior:

# Target Roombas
- Every 20 seconds, 180 deg turn clockwise
- Every 5 seconds, +-20 deg turn
- Front collision -> 180 deg turn clockwise
- Top collision -> 45 deg turn clockwise

# Obstacle Roombas
- Move in a circular motion around the origin
- Front collision -> idle until unobstructed
'''

import numpy as np
import random

import roombasim.config as cfg

class Roomba(object):
    '''
    Represents a generic roomba.
    (No update function)
    '''

    def __init__(self, pos, heading, tag=None):
        '''
        Initialize a roomba object with a given position and heading.
        By default, the roomba starts in STATE_IDLE.

        pos - [x,y] in meters
        heading - angle in radians (0 is +x and pi/2 is +y)
        [tag] - an optional identification element
        '''
        self.pos = pos
        self.heading = heading
        self.tag = tag

        self.collisions = {
            'front': False,
            'top': False
        }

        self.timers = {
            'reverse': 0,
            'noise': 0,
            'touch': 0
        }

        self.state = cfg.ROOMBA_STATE_IDLE

        # amount we need to turn
        self.turn_target = 0
        self.turn_clockwise = False

    def start(self):
        '''
        Sets the roomba to STATE_FORWARD
        '''
        self.state = cfg.ROOMBA_STATE_FORWARD

    def stop(self):
        '''
        Sets the roomba to STATE_IDLE

        Note: if the roomba was mid-turn during this call, it will
        not resume after a restart.
        '''
        self.state = cfg.ROOMBA_STATE_IDLE

    def update(self, delta, elapsed):
        '''
        Perform an update step (unimplemented)
        '''
        pass


class TargetRoomba(Roomba):
    '''
    Represents a target roomba.
    '''

    def update(self, delta, elapsed):
        '''
        Perform an update step.

        delta - change in time since last update (seconds)
        elapsed - total time elapsed since start (milliseconds)
        '''
        if self.state == cfg.ROOMBA_STATE_FORWARD:
            if self.collisions['top']:
                self.state = cfg.ROOMBA_STATE_TOUCHED
                self.collisions['top'] = False
                self.timers['touch'] = elapsed
            elif elapsed - self.timers['reverse'] > cfg.ROOMBA_REVERSE_PERIOD:
                self.state = cfg.ROOMBA_STATE_REVERSING
                self.timers['reverse'] = elapsed
            elif elapsed - self.timers['noise'] > cfg.ROOMBA_HEADING_NOISE_PERIOD:
                self.state = cfg.ROOMBA_STATE_TURNING_NOISE
                self.angular_noise_velocity = (random.uniform(-cfg.ROOMBA_HEADING_NOISE_MAX,
                                                              cfg.ROOMBA_HEADING_NOISE_MAX)
                                             / (cfg.ROOMBA_NOISE_DURATION / 1000.0))
                self.timers['noise'] = elapsed
            elif self.collisions['front']:
                self.collisions['front'] = False
                self.state = cfg.ROOMBA_STATE_REVERSING
                self.timers['reverse'] = elapsed
            else:
                self.pos[0] += cfg.ROOMBA_LINEAR_SPEED * np.cos(self.heading) * delta
                self.pos[1] += cfg.ROOMBA_LINEAR_SPEED * np.sin(self.heading) * delta
        elif self.state == cfg.ROOMBA_STATE_TOUCHED:
            self.collisions['top'] = False #TODO: Is this right?
            turn_time = (np.pi / 4) / cfg.ROOMBA_ANGULAR_SPEED
            if elapsed - self.timers['touch'] >= turn_time * 1000:
                self.state = cfg.ROOMBA_STATE_FORWARD
            elif self.collisions['front']:
                self.state = cfg.ROOMBA_STATE_REVERSING
                self.collisions['front'] = False
                self.timers['reverse'] = elapsed
            else:
                self.heading -= cfg.ROOMBA_ANGULAR_SPEED * delta
        elif self.state == cfg.ROOMBA_STATE_REVERSING:
            self.collisions['front'] = False #TODO: Is this right?
            if self.collisions['top']:
                self.collisions['top'] = False
                self.state = cfg.ROOMBA_STATE_TOUCHED
                self.timers['touch'] = elapsed
            elif elapsed - self.timers['reverse'] >= np.pi / cfg.ROOMBA_ANGULAR_SPEED * 1000:
                self.state = cfg.ROOMBA_STATE_FORWARD
            else:
                self.heading -= cfg.ROOMBA_ANGULAR_SPEED * delta
            self.collisions['front'] = False
        elif self.state == cfg.ROOMBA_STATE_TURNING_NOISE:
            if self.collisions['top']:
                self.collisions['top'] = False
                self.state = cfg.ROOMBA_STATE_TOUCHED
                self.timers['touch'] = elapsed
            elif elapsed - self.timers['noise'] >= cfg.ROOMBA_NOISE_DURATION:
                self.state = cfg.ROOMBA_STATE_FORWARD
            elif self.collisions['front']:
                self.collisions['front'] = False
                self.state = cfg.ROOMBA_STATE_REVERSING
                self.timers['reverse'] = elapsed
            else:
                self.heading += self.angular_noise_velocity * delta
                self.pos[0] += cfg.ROOMBA_LINEAR_SPEED * np.cos(self.heading) * delta
                self.pos[1] += cfg.ROOMBA_LINEAR_SPEED * np.sin(self.heading) * delta
        elif self.state != cfg.ROOMBA_STATE_IDLE:
            assert False

class ObstacleRoomba(Roomba):
    '''
    Represents an obstacle roomba.
    '''

    def update(self, delta, elapsed):
        '''
        Perform an update step.

        Note: in order to acheive circular motion, the heading is
        updated each step to be perpendicular to a vector from
        the center to the roomba. For small deltas, this should
        provide realistic behavior.

        delta - change in time since last update (seconds)
        elapsed - total time elapsed since start (milliseconds)
        '''
        if self.collisions['front']:
            self.collisions['front'] = False
        elif self.state == cfg.ROOMBA_STATE_FORWARD:
            self.pos[0] += cfg.ROOMBA_LINEAR_SPEED * np.cos(self.heading) * delta
            self.pos[1] += cfg.ROOMBA_LINEAR_SPEED * np.sin(self.heading) * delta

            # reorient so we tangent to a circle centered at the origin
            ang = np.arctan2(10 - self.pos[1], 10 - self.pos[0])
            self.heading = ang + (cfg.PI / 2)
