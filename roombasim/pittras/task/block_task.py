'''
block_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

from roombasim.pittras.task import BlockRoombaTask

class BlockTask(Task):
    '''
    A task to zero lateral velocity and land for a specified amount of time.

    Use this task if you are able to predict the blocking position of a roomba.

    If a target roomba is supplied, the drone will rotate to become perpendicular to the
    target roomba's heading like the BlockRoombaTask.
    '''

    def __init__(self, block_duration, target_roomba=None):
        '''
        block_duration - how long to remain on the ground in seconds
        '''
        self.block_duration = block_duration
        self.target_roomba = target_roomba

        self.land_time = None

        # PID controllers
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)
        self.pid_yaw = PIDController(cfg.PITTRAS_PID_YAW)

    def update(self, delta, elapsed, state_controller, environment):
        # fetch drone odometry
        drone_state = state_controller.query('DroneState', environment)

        if self.land_time is not None:
            if elapsed - self.land_time > self.block_duration:
                self.complete(TaskState.SUCCESS)
            return

        if drone_state['z_pos'] == 0:
            self.land_time = elapsed

        # PID calculations
        control_xy = self.pid_xy.get_control(
            np.array([0, 0]),
            -drone_state['xy_vel'],
            delta
        )

        control_yaw = 0

        if self.target_roomba is not None:
            # fetch roomba odometry
            target_roombas, _ = state_controller.query('RoombaState', environment)

            self.target_yaw = BlockRoombaTask._calculate_target_yaw(
                drone_state['yaw'],
                target_roombas[self.target_roomba]['heading']
            )

            control_yaw = self.pid_yaw.get_control(
                self.target_yaw - drone_state['yaw'],
                -drone_state['yaw'],
                delta
            )

        # normalize acceleration vector
        adjusted_xy = geometry.rotate_vector(control_xy, -drone_state['yaw'])

        # perform control action
        environment.agent.control(adjusted_xy,
                                  control_yaw,
                                  cfg.PITTRAS_BLOCK_DESCENT_VEL)
