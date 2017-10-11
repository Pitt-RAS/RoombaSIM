'''
drone_state.py

Contains the DroneState odometry sensor.
'''
from roombasim.ai import State

class DroneState(State):
    '''
    Drone odometry

    {
        'xy_pos': float[2]
        'xy_vel': float[2]
        'z_pos': float
        'z_vel': float
        'yaw': float
        'yaw_vel': float
    }
    '''

    @staticmethod
    def query(environment):
        agent = environment.agent

        state = {
            'xy_pos': agent.xy_pos,
            'xy_vel': agent.xy_vel,
            'z_pos': agent.z_pos,
            'z_vel': agent.z_vel,
            'yaw': agent.yaw,
            'yaw_vel': agent.yaw_vel
        }

        return state
