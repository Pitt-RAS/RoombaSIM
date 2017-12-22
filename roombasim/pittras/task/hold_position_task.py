'''
hold_position_task.py
'''

import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState
from roombasim.pid_controller import PIDController
from roombasim import geometry

class HoldPositionTaskStates:
    '''
    States for the Hold Position task.
    '''
    init = 0
    holding = 1
    done = 2
    failed = 3


class HoldPositionTask(Task):
    '''
    A task to hold the drone at its current absolute position for some duration
    of time. The task accepts a float64 that specifies such duration in seconds,
    which means holding the current position indefinitely if less than or equal
    to zero.
    '''

    def __init__(self, hold_duration):
        self.hold_duration = hold_duration
        self.state = HoldPositionTaskStates.init

        # Hold position properties
        self.start_time = 0
        self.hold_xy = np.array([0, 0])
        self.hold_z = 0

        # PID controllers
        self.pid_xy = PIDController(cfg.PITTRAS_PID_XY, dimensions=2)
        self.pid_z = PIDController(cfg.PITTRAS_PID_Z)

    def update(self, delta, elapsed, state_controller, environment):
        if (self.state == HoldPositionTaskStates.done or
                self.state == HoldPositionTaskStates.failed):
            return

        # Fetch current odometry
        drone_state = state_controller.query('DroneState', environment)

        if self.state == HoldPositionTaskStates.init:
            # TODO(zacyu): Remove this when `elapsed` is changed to represent
            #              the elipsed time since the start of the task.
            self.start_time = elapsed
            self.state = HoldPositionTaskStates.holding
            self.hold_xy = drone_state['xy_pos']
            self.hold_z = drone_state['z_pos']
            return

        if (self.hold_duration > 0) and (elapsed - self.start_time >
                                         self.hold_duration * 1000):
            if (np.linalg.norm(drone_state['xy_pos'] - self.hold_xy) <
                    cfg.PITTRAS_HOLD_POSITION_TOLERANCE and
                    abs(drone_state['z_pos'] - self.hold_z) <
                    cfg.PITTRAS_HOLD_POSITION_TOLERANCE):
                self.complete(TaskState.SUCCESS)
                self.state = HoldPositionTaskStates.done
            else:
                self.complete(TaskState.FAILURE)
                self.state = HoldPositionTaskStates.failed

        # PID calculations
        control_xy = self.pid_xy.get_control(
            self.hold_xy - drone_state['xy_pos'],
            -drone_state['xy_vel'],
            delta
        )

        control_z = self.pid_z.get_control(
            self.hold_z - drone_state['z_pos'],
            -drone_state['z_vel'],
            delta
        )

        # normalize acceleration vector
        adjusted_xy = geometry.rotate_vector(control_xy, -drone_state['yaw'])

        # Perform control action
        environment.agent.control(adjusted_xy, 0, control_z)
