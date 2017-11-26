
import numpy as np

from roombasim.ai import Task, TaskState

import roombasim.config as cfg
from roombasim import geometry

class BlockRoombaTask(Task):

    def __init__(self, target_roomba, block_vector):
        '''
        target_roomba - target roomba tag
        block_vector - where to land with respect to the roomba
        '''
        self.target_roomba = target_roomba
        self.block_vector = block_vector

        # calculated on first update
        self.target_yaw = None
        self.target_xy = None

        # PID controller contstants
        self.k_xy = cfg.PITTRAS_PID_XY
        self.k_z = cfg.PITTRAS_PID_Z
        self.k_yaw = cfg.PITTRAS_PID_YAW

        self.i_xy = np.array([0, 0], dtype=np.float64)
        self.i_z = 0
        self.i_yaw = 0

        # estimate roomba velocity
        self.last_target_xy = None

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
        if self.target_yaw is None:
            self.target_yaw = BlockRoombaTask._calculate_target_yaw(
                drone_state['yaw'],
                roomba['heading']
            )

        if self.target_xy is None:
            self.target_xy = np.array(roomba['pos']) + self.block_vector

        # if np.linalg.norm(target_xy - drone_state['xy_pos']) < cfg.PITTRAS_XYZ_TRANSLATION_ACCURACY:
        #     self.complete(TaskState.SUCCESS)
        #     return

        # xy PID controller
        p_xy = (self.target_xy - drone_state['xy_pos'])
        d_xy = -drone_state['xy_vel']
        self.i_xy += p_xy * delta
        control_xy = self.k_xy.dot([p_xy, d_xy, self.i_xy])

        # rotate the control xy vector counterclockwise about
        # the origin by the current yaw
        yaw = -drone_state['yaw']
        rot_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])
        adjusted_xy = rot_matrix.dot(control_xy)

        # z PID controller
        p_z = - drone_state['z_pos']
        d_z = - drone_state['z_vel']
        self.i_z += p_z * delta
        control_z = self.k_z.dot([p_z, d_z, self.i_z])

        # yaw PID controller
        p_yaw = (self.target_yaw - drone_state['yaw'])
        d_yaw = - drone_state['yaw_vel']
        self.i_yaw += p_yaw * delta
        control_yaw = self.k_yaw.dot([p_yaw, d_yaw, self.i_yaw])

        # self.last_target_xy = target_xy

        # perform control action
        environment.agent.control(adjusted_xy, control_yaw, control_z)


    @staticmethod
    def _calculate_target_yaw(drone_heading, roomba_heading):
        angle_diff = (drone_heading - roomba_heading) % (np.pi / 2)

        if angle_diff < (np.pi / 4):
            return drone_heading - angle_diff
        else:
            return drone_heading - angle_diff + (np.pi / 2)
