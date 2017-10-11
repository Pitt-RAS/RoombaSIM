
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task

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

    def __init__(self):
        # import constants
        self._TAKEOFF_VELOCITY = cfg.PITTRAS_TAKEOFF_VELOCITY
        self._TAKEOFF_COMPLETE_HEIGHT = cfg.PITTRAS_TAKEOFF_COMPLETE_HEIGHT
        self._ANGLE_MODE_HEIGHT = cfg.PITTRAS_TAKEOFF_ANGLE_MODE_HEIGHT
        self._DELAY_BEFORE_TAKEOFF = cfg.PITTRAS_DELAY_BEFORE_TAKEOFF
        self._TRANSFORM_TIMEOUT = cfg.PITTRAS_TAKEOFF_TRANSFORM_TIMEOUT

        # null velocities
        self.zero_xy_vel = np.array([0,0], dtype=np.float64)

        # state machine
        self._state = TakeoffTaskState.init

    def update(self, delta, elapsed, state_controller, environment):
        drone_state = state_controller.query('DroneState', environment)

        height = drone_state['z_pos']
        control_z_vel = drone_state['z_vel']

        # Pause before ramping up the motors
        if elapsed > self._DELAY_BEFORE_TAKEOFF * 1000:
            # Check if we reached the target height
            if height < self._TAKEOFF_COMPLETE_HEIGHT:
                # Ascend if below the target height
                self._state = TakeoffTaskState.ascend

                control_z_vel = cfg.PITTRAS_TAKEOFF_VELOCITY
            else:
                # Stop ascending if above the target height
                self._state = TakeoffTaskState.done

                control_z_vel = 0

        print("delta: " + str(delta) + ", elapsed: " + str(elapsed) + ", control_z: " + str(control_z))
        environment.agent.control(self.zero_xy_vel, 0, control_z_vel)
