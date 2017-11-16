'''
xyz_translation_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState

class XYZTranslationTask(Task):
    '''
    A task to move the drone to an absolute position somewhere
    in the arena. The task accepts a single 3d vector that defines
    the target position as an [x,y,z] coordinate. Two PID controllers
    are used: one for the xy-plane and one for the z-axis, in order
    to control the drone's position.
    '''

    def __init__(self, target):
        '''
        target - 3d (x,y,z) vector
        '''
        self.target_xy = np.array(target[:2])
        self.target_z = target[2]

        # PID controller contstants
        self.k_xy = cfg.PITTRAS_PID_XY
        self.k_z = cfg.PITTRAS_PID_Z

        self.i_xy = np.array([0,0], dtype=np.float64)
        self.i_z = 0

    def update(self, delta, elapsed, state_controller, environment):
        # fetch current odometry
        drone_state = state_controller.query('DroneState', environment)

        dist = np.linalg.norm(self.target_xy - drone_state['xy_pos'])

        if dist < cfg.PITTRAS_XYZ_TRANSLATION_ACCURACY:
            self.complete(TaskState.SUCCESS)
            return

        # xy PID controller
        p_xy = (self.target_xy - drone_state['xy_pos'])
        d_xy = - drone_state['xy_vel']
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
        p_z = (self.target_z - drone_state['z_pos'])
        d_z = - drone_state['z_vel']
        self.i_z += p_z * delta
        control_z = self.k_z.dot([p_z, d_z, self.i_z])

        # perform control action
        environment.agent.control(adjusted_xy, 0, control_z)
