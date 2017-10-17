
import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState

class LandTaskState(object):
    init = 0
    descend = 1
    done = 2

class LandTask(Task):

    def __init__(self):
        # state machine
        self._state = LandTaskState.init

    def update(self, delta, elapsed, state_controller, environment):
        drone_state = state_controller.query('DroneState', environment)

        height = drone_state['z_pos']

        # Check if we reached the target landing height tolerance
        if height > cfg.PITTRAS_LAND_HEIGHT_TOLERANCE:
            # Descend if above the landing height tolerance
            self._state = LandTaskState.descend

            environment.agent.control([0,0], 0, cfg.PITTRAS_LAND_VELOCITY)
        else:
            # Stop descending if below the target height tolerance
            self._state = LandTaskState.done

            environment.agent.control([0,0], 0, 0)

            self.complete(TaskState.SUCCESS)
