'''
block_roomba_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

class BlockRoombaTask(Task):
    '''
    A task that lands in front of a roomba.
    '''

    def __init__(self, target_roomba, block_vector):
        '''
        target_roomba - target roomba tag
        block_vector - float[2] that specifies where to land with respect to the
            the target roomba
        '''
        self.target_roomba = target_roomba
        self.block_vector = np.array(block_vector)

        # calculated on first update
        self.target_yaw = None
        self.target_xy = None

        # PID controllers
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)
        self.pid_yaw = PIDController(cfg.PITTRAS_PID_YAW)

    def update(self, delta, elapsed, state_controller, environment):
        # fetch roomba odometry
        target_roombas, _ = state_controller.query('RoombaState', environment)

        # fetch drone odometry
        drone_state = state_controller.query('DroneState', environment)

        if not self.target_roomba in target_roombas:
            self.complete(TaskState.FAILURE, "Roomba not found")
            return

        roomba = target_roombas[self.target_roomba]

        # calculate the target yaw so that we turn less than 45 degrees
        # in either direction and land with a bumper side perpendicular
        # to the direction of roomba motion
        self.target_yaw = BlockRoombaTask._calculate_target_yaw(
            drone_state['yaw'],
            roomba['heading']
        )

        # calculate the target landing position
        block_vector = geometry.rotate_vector(self.block_vector,
                                              roomba['heading'])
        self.target_xy = np.array(roomba['pos']) + block_vector

        # PID calculations
        control_xy = self.pid_xy.get_control(
            self.target_xy - drone_state['xy_pos'],
            -drone_state['xy_vel'],
            delta
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

    @staticmethod
    def _calculate_target_yaw(drone_heading, roomba_heading):
        '''
        Returns a yaw within 45 degrees of the current drone heading
        such that one of the four faces is perpendicular to the roomba.
        '''
        angle_diff = (drone_heading - roomba_heading) % (np.pi / 2)

        if angle_diff < (np.pi / 4):
            return drone_heading - angle_diff
        else:
            return drone_heading - angle_diff + (np.pi / 2)
