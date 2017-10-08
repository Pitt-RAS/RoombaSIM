
from roombasim.ai import State

class DroneState(State):

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
