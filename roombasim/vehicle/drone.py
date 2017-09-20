'''
drone.py

Generic Drone implementation
'''

class Drone(object):
    '''
    Extend this generic class to provide team
    specific implementation details.
    '''

    def __init__(self, pos, heading):
        # position of center point
        self.pos = pos

        # yaw direction
        self.heading = heading

        # distance from the ground
        # (when landed, this should be zero)
        self.altitude = 0

    # The following functions should be implemented by
    # a team-specific subclass:

    def update(self, delta, elapsed):
        '''
        Perform a physics update step
        '''
        raise NotImplementedError

    def is_touching_roomba_top(self, rba):
        '''
        True if the drone is making contact with roomba top switch.
        '''
        raise NotImplementedError

    def is_blocking_roomba(self, rba):
        '''
        True if drone is making contact with roomba front bumper.
        '''
        raise NotImplementedError
