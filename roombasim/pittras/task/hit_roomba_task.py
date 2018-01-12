'''
hit_roomba_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

class HitRoombaTask(Task):
    '''
    A task to bump the top of a roomba.
    '''

    def __init__(self, target_roomba):
        self.target_roomba = target_roomba

        # PID controllers
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)

        # estimate roomba velocity
        self.last_target_xy = None

        self.land_time = None

    def update(self, delta, elapsed, state_controller, environment):
        # fetch roomba odometry
        target_roombas, _ = state_controller.query('RoombaState', environment)

        # fetch drone odometry
        drone_state = state_controller.query('DroneState', environment)

        if not self.target_roomba in target_roombas:
            self.complete(TaskState.FAILURE, "Roomba not found")
            return

        roomba = target_roombas[self.target_roomba]

        target_xy = np.array(roomba['pos'])

        # check if the roomba is too far away
        if np.linalg.norm(target_xy - drone_state['xy_pos']) > cfg.PITTRAS_HIT_ROOMBA_MAX_START_DIST:
            self.complete(TaskState.FAILURE, "Roomba is too far away")
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

        # normalize acceleration vector
        adjusted_xy = geometry.rotate_vector(control_xy, -drone_state['yaw'])

        # perform control action
        environment.agent.control(adjusted_xy,
                                  0,
                                  cfg.PITTRAS_HIT_ROOMBA_DESCENT_VELOCITY)

        # check if we have hit the roomba
        if drone_state['z_pos'] < cfg.PITTRAS_DRONE_PAD_ACTIVIATION_HEIGHT:
            if self.land_time is None:
                self.land_time = elapsed
            if elapsed - self.land_time > cfg.PITTRAS_HIT_ROOMBA_FLOOR_TIME * 1000:
                self.complete(TaskState.SUCCESS)
            return

        # update delayed trackers
        self.last_target_xy = target_xy
