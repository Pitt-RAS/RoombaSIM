'''
hold_position_task.py
'''

import numpy as np

from roombasim.ai import Task

class HoldPositionTaskStates:
    '''
    States for the Hold Position task.
    '''
    init = 0
    holding = 1
    done = 2

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
        self.start_time = 0

    def update(self, delta, elapsed, state_controller, environment):
        if self.done == HoldPosition.done:
            # TODO(zacyu): Signal the termination of the task when such
            #              capability is added to the framework.
            return

        if self.state == HoldPositionTaskStates.init:
            # TODO(zacyu): Remove this when `elapsed` is changed to represent
            #              the elipsed time since the start of the task.
            self.start_time = elapsed
            self.state = HoldPositionTaskStates.holding
            return

        if (self.hold_duration > 0) and (elapsed - self.start_time >
                                         self.hold_duration * 1000):
            self.state = HoldPositionTaskStates.done
