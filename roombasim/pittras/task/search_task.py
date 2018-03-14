'''
search_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

class SearchTask(Task):

    def __init__(self, target, trigger_radius, timeout):
        '''
        Move the drone to an arbitrary position and return when a roomba enters within the
        trigger radius or after a timeout occurs.

        target - (x, y) position of search space
        trigger_radius - minimum distance to a roomba in order to terminate (in meters)
        timeout - timeout before aborting the task (in seconds)
        '''
        self.target_xy = np.array(target)
        self.target_vel = np.array([0,0])
        self.trigger_radius = trigger_radius
        self.timeout = timeout

        self._init_time = None

        # PID controllers
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)
        self.pid_z = PIDController(cfg.PITTRAS_PID_Z)

    def update(self, delta, elapsed, state_controller, environment):

        # TODO: replace when elapsed become time from start of task
        if self._init_time is None:
            self._init_time = elapsed

        # check for timeout
        elif elapsed - self._init_time > self.timeout:
            self.complete(TaskState.SUCCESS)

        # fetch roomba odometry
        target_roombas, _ = state_controller.query('RoombaState', environment)

        # fetch drone odometry
        drone_state = state_controller.query('DroneState', environment)

        # check if any roombas are within the radius
        for tag in target_roombas:
            r = target_roombas[tag]
            dist = np.linalg.norm(self.target_xy - r['pos'])
            if dist < self.trigger_radius:
                self.complete(TaskState.SUCCESS)
                return

        # PID calculations
        control_xy = self.pid_xy.get_control(
            self.target_xy - drone_state['xy_pos'],
            self.target_vel - drone_state['xy_vel'],
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
