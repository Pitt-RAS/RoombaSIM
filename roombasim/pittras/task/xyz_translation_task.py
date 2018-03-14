'''
xyz_translation_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

from roombasim.pittras.task.block_roomba_task import BlockRoombaTask

class XYZTranslationTask(Task):
    '''
    A task to move the drone to an absolute position somewhere
    in the arena. The task accepts a single 3d vector that defines
    the target position as an [x,y,z] coordinate. Two PID controllers
    are used: one for the xy-plane and one for the z-axis, in order
    to control the drone's position.

    The task terminates with SUCCESS when the drone is within
    PITTRAS_XYZ_TRANSLATION_ACCURACY meters of the target position.
    '''

    def __init__(self, target, hold_position=False, normalize_yaw=False):
        '''
        target - 3d (x,y,z) vector
        hold_position - a boolean describing whether the task should terminate once the drone has
            reached the target position
        '''
        self.target_xy = np.array(target[:2])
        self.target_z = target[2]
        self.hold_position = hold_position
        self.normalize_yaw = normalize_yaw

        # PID controllers
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)
        self.pid_z = PIDController(cfg.PITTRAS_PID_Z)
        self.pid_yaw = PIDController(cfg.PITTRAS_PID_YAW)

    def set_target(self, target):
        self.target_xy = np.array(target[:2])
        self.target_z = target[2]

    def update(self, delta, elapsed, state_controller, environment):
        # fetch current odometry
        drone_state = state_controller.query('DroneState', environment)

        # check if we have reached the target
        if not self.hold_position:
            dist = np.linalg.norm(self.target_xy - drone_state['xy_pos'])
            if dist < cfg.PITTRAS_XYZ_TRANSLATION_ACCURACY:
                self.complete(TaskState.SUCCESS)
                return

        # PID calculations
        control_xy = self.pid_xy.get_control(
            self.target_xy - drone_state['xy_pos'],
            - drone_state['xy_vel'],
            delta
        )

        control_z = self.pid_z.get_control(
            self.target_z - drone_state['z_pos'],
            - drone_state['z_vel'],
            delta
        )

        control_yaw = 0
        if self.normalize_yaw:
            target_yaw = BlockRoombaTask._calculate_target_yaw(drone_state['yaw'], 0)

            control_yaw = self.pid_yaw.get_control(
                target_yaw - drone_state['yaw'],
                - drone_state['yaw_vel'],
                delta
            )

        # rotate the control xy vector counterclockwise about
        # the origin by the current yaw
        adjusted_xy = geometry.rotate_vector(control_xy, -drone_state['yaw'])

        # perform control action
        environment.agent.control(adjusted_xy, control_yaw, control_z)
