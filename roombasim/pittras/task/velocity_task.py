'''
velocity_task.py
'''

import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

class VelocityTask(Task):
    '''
    A task to accelerate/decelerate the drone to a given velocity. The task
    accepts a single 3d vector that defines the target velocty.
    '''

    def __init__(self, target):
        '''
        target - 3d (v_x, v_y, v_z) vector
        '''
        self.target_xy = np.array(target[:2])
        self.target_z = target[2]

        # PID controller
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)

    def update(self, delta, elapsed, state_controller, environment):
        # Fetch current odometry
        drone_state = state_controller.query('DroneState', environment)

        if np.linalg.norm(self.target_xy - drone_state['xy_vel']) < \
                cfg.PITTRAS_VELOCITY_TOLERANCE:
            self.complete(TaskState.SUCCESS)
            return

        # Calculate control acceleration vector
        control_xy = self.pid_xy.get_control(
            self.target_xy - drone_state['xy_vel'],
            [0, 0],
            delta
        )

        # normalize acceleration vector
        adjusted_xy = geometry.rotate_vector(control_xy, -drone_state['yaw'])

        # Perform control action
        environment.agent.control(adjusted_xy, 0, self.target_z)
