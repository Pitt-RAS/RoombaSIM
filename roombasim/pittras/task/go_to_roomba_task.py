'''
go_to_roomba_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

class GoToRoombaTask(Task):

    def __init__(self, target_roomba, offset_xy):
        '''
        target_roomba - target roomba tag
        offset_xy - float[2] that defines x and y offset from the roomba
        '''
        self.target_roomba = target_roomba
        self.offset_xy = offset_xy

        # PID controllers
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)
        self.pid_z = PIDController(cfg.PITTRAS_PID_Z)

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

        adjusted_offset_xy = geometry.rotate_vector(self.offset_xy, roomba['heading'])

        target_xy = np.array(roomba['pos']) + adjusted_offset_xy

        if np.linalg.norm(target_xy - drone_state['xy_pos']) < cfg.PITTRAS_XYZ_TRANSLATION_ACCURACY:
            self.complete(TaskState.SUCCESS)
            return

        if self.last_target_xy is None:
            self.last_target_xy = target_xy

        target_vel_xy = ((target_xy - self.last_target_xy) / delta)

        # PID calculations
        control_xy = self.pid_xy.get_control(
            target_xy - drone_state['xy_pos'],
            target_vel_xy - drone_state['xy_vel'],
            delta
        )

        control_z = self.pid_z.get_control(
            cfg.PITTRAS_TRACK_ROOMBA_HEIGHT - drone_state['z_pos'],
            -drone_state['z_vel'],
            delta
        )

        # normalize acceleration vector
        adjusted_xy = geometry.rotate_vector(control_xy, -drone_state['yaw'])

        # perform control action
        environment.agent.control(adjusted_xy, 0, control_z)

        # update delayed trackers
        self.last_target_xy = target_xy
