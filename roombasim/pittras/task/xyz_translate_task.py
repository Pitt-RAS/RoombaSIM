
import numpy as np

from roombasim.ai import Task

class XYZTranslateTask(Task):
    
    def __init__(self, target):
        '''
        target - 3d (x,y,z) vector
        yaw - target yaw angle
        '''
        self.target_xy = np.array(target[:2])
        self.target_z = target[2]

        # simple PID controller
        self.k_xy = np.array([0.5,1.1,0])
        self.k_z = np.array([0.5,1.1,0])

        self.i_xy = np.array([0,0], dtype=np.float64)
        self.i_z = 0

    def update(self, delta, elapsed, state_controller, environment):
        drone_state = state_controller.query('DroneState', environment)

        # xy PID controller
        p_xy = (self.target_xy - drone_state['xy_pos'])
        d_xy = - drone_state['xy_vel']
        self.i_xy += p_xy * delta
        control_xy = self.k_xy.dot([p_xy, d_xy, self.i_xy])

        # rotate the control xy vector about the origin
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

        environment.agent.control(adjusted_xy, 0, control_z)

