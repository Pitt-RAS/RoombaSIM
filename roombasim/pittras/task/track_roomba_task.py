'''
track_roomba_task.py
'''
import numpy as np

from roombasim.ai import Task, TaskState

import roombasim.config as cfg
from roombasim.pid_controller import PIDController
from roombasim import geometry

class TrackRoombaTask(Task):
    '''
    A task to follow a roomba with an optional offset vector. If the supplied timeout
    duration is less than or equal to zero, the task will run indefinitely.

    The drone will attempt to maintain a height of PITTRAS_TRACK_ROOMBA_HEIGHT while
    tracking the roomba.
    '''

    def __init__(self, target_roomba, offset_xy, timeout):
        '''
        target_roomba - target roomba tag
        offset_xy - float[2] that defines x and y offset from the roomba
        timeout - task duration in milliseconds. If less than or equal to zero,
            the task will run indefinitely
        '''
        self.target_roomba = target_roomba
        self.offset_xy = offset_xy
        self.timeout = timeout

        # TODO(hgarrereyn): remove once tasks get per-task elapsed time rather than global
        self.start_time = None

        # PID controllers
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)
        self.pid_z = PIDController(cfg.PITTRAS_PID_Z)

        # estimate roomba velocity
        self.last_target_xy = None

    def update(self, delta, elapsed, state_controller, environment):
        # first update
        if self.start_time is None:
            self.start_time = elapsed

        # fetch roomba odometry
        target_roombas, _ = state_controller.query('RoombaState', environment)

        # fetch drone odometry
        drone_state = state_controller.query('DroneState', environment)

        if not self.target_roomba in target_roombas:
            self.complete(TaskState.FAILURE, "Roomba not found")
            return

        # calculate xy target position
        roomba = target_roombas[self.target_roomba]
        target_xy = np.array(roomba['pos']) + self.offset_xy

        # check if we should timeout
        if self.timeout > 0 and elapsed - self.start_time >= self.timeout:
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
            - drone_state['z_vel'],
            delta
        )

        # normalize acceleration vector
        adjusted_xy = geometry.rotate_vector(control_xy, -drone_state['yaw'])

        self.last_target_xy = target_xy

        # perform control action
        environment.agent.control(adjusted_xy, 0, control_z)
