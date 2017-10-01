from roombasim.environment import State

class OdometryState(State):
    '''
    Drone status sensor.

    Returns:
    {
        'xy_pos': [x,y],
        'xy_vel': 2d vector,
        'xy_accel': 
        'altitude': a,

    }
    '''

    @staticmethod
    def query(environment):
        agent = environment.agent

        return {
            'xy_pos': agent.xy_pos,
            'xy_vel': agent.xy_vel,
            'xy_accel': agent.xy_accel
        }

        