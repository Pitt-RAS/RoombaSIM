
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task

class LandTaskState:
    init = 0
    descend = 1
    descend_angle = 2
    done = 3
    failed = 4

class LandTask(Task):

    def __init__(self):
        # state machine
        self._state = TakeoffTaskState.init

    def update(self, delta, elapsed, state_controller, environment):
        drone_state = state_controller.query('DroneState', environment)

        height = drone_state['z_pos']
        control_z_vel = 0

        # Check if we reached the target landing height tolerance
        if height > cfg.PITTRAS_LAND_HEIGHT_TOLERANCE:
            # Descend if above the landing height tolerance
            self._state = LandTaskState.descend

            control_z_vel = cfg.PITTRAS_LAND_VELOCITY
        else:
            # Stop descending if below the target height tolerance
            self._state = LandTaskState.done

            self.complete(Task.SUCCESS)
            return

        environment.agent.control(self.zero_xy_vel, 0, control_z_vel)
