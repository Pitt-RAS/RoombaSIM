'''
takeoff_task.py
'''
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState

class TakeoffTaskState:
    init = 0
    request_arm = 1
    pause_before_takeoff = 2
    takeoff = 3
    ascend = 4
    ascend_angle = 5
    done = 6
    failed = 7

class TakeoffTask(Task):
    '''
    A task to control drone takeoff.

    The drone will wait for PITTRAS_DELAY_BEFORE_TAKEOFF seconds before
    accelerating upwards with a target velocity of PITTRAS_TAKEOFF_VELOCITY
    until reaching a height of PITTRAS_TAKEOFF_COMPLETE_HEIGHT at which point
    the task will terminate with SUCCESS.
    '''

    def __init__(self):
        # null velocities
        self.zero_xy_vel = np.array([0,0], dtype=np.float64)

        # state machine
        self._state = TakeoffTaskState.init

    def update(self, delta, elapsed, state_controller, environment):
        drone_state = state_controller.query('DroneState', environment)

        height = drone_state['z_pos']
        control_z_vel = drone_state['z_vel']

        # Pause before ramping up the motors
        if elapsed > cfg.PITTRAS_DELAY_BEFORE_TAKEOFF * 1000:
            # Check if we reached the target height
            if height < cfg.PITTRAS_TAKEOFF_COMPLETE_HEIGHT:
                # Ascend if below the target height
                self._state = TakeoffTaskState.ascend

                control_z_vel = cfg.PITTRAS_TAKEOFF_VELOCITY
            else:
                # Stop ascending if above the target height
                self._state = TakeoffTaskState.done

                control_z_vel = 0

                self.complete(TaskState.SUCCESS)

        environment.agent.control(self.zero_xy_vel, 0, control_z_vel)
