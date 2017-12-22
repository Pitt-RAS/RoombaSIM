'''
roomba_state.py

Provides a RoombaState class to represent roomba detection.
'''

from roombasim.ai import State
from roombasim.environment import TargetRoomba, ObstacleRoomba

class RoombaState(State):
    '''
    Roomba odometry:

    Returns (target_roombas, obstacle_roombas) where both dicts
    are of type:

    {
        tag : {
            pos : float[2],
            heading : float
        }
        ...
    }
    '''

    @staticmethod
    def query(environment):
        target_roombas = {}
        obstacle_roombas = {}

        for r in environment.roombas:
            if r.tag is not None:
                if isinstance(r, TargetRoomba):
                    target_roombas[r.tag] = {
                        'pos': r.pos,
                        'heading': r.heading
                    }
                elif isinstance(r, ObstacleRoomba):
                    obstacle_roombas[r.tag] = {
                        'pos': r.pos,
                        'heading': r.heading
                    }

        return (target_roombas, obstacle_roombas)
