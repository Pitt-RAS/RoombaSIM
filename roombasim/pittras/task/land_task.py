'''
land_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

class LandTaskState(object):
    init = 0
    descend = 1
    done = 2

class LandTask(Task):
    '''
    A task to land the drone.
    '''

    def __init__(self):
        # state machine
        self._state = LandTaskState.init

        # xy PID controller for velocity control
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)

    def update(self, delta, elapsed, state_controller, environment):
        # fetch drone state
        drone_state = state_controller.query('DroneState', environment)
        height = drone_state['z_pos']

        # we don't care about position but the horizontal target velocity is zero
        control_xy = self.pid_xy.get_control(
            np.zeros(2),
            - drone_state['xy_vel'],
            delta
        )
        adjusted_xy = geometry.rotate_vector(control_xy, -drone_state['yaw'])

        # Check if we reached the target landing height tolerance
        # TODO: if landing sensors are provided via the state controller
        # then use them to determine if the drone has landed
        if height > 0:
            # Descend if above the landing height tolerance
            self._state = LandTaskState.descend

            environment.agent.control(adjusted_xy, 0, cfg.PITTRAS_LAND_VELOCITY)
        else:
            # Stop descending if below the target height tolerance
            self._state = LandTaskState.done
            self.complete(TaskState.SUCCESS)
