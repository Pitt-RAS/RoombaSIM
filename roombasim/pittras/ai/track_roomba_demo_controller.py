'''
track_roomba_demo_controller.py
'''
from roombasim.ai import Controller

import roombasim.config as cfg

class TrackRoombaDemoController(Controller):
    '''
    A demonstration of the GoToRoomba and TrackRoomba tasks.

    Every 8 seconds, the drone will fly to a new target and track it.
    '''

    def setup(self):
        # initial target
        self.target = 0

        # set the initial target
        self.go_to(self.target)

        self.last_switch = 0

    def go_to(self, roomba):
        print("Going to: " + str(roomba))
        self.task_controller.switch_task(
            'GoToRoombaTask',
            target_roomba = roomba,
            offset_xy = [0, 0],
            callback=(lambda a,b: self.track(roomba))
        )

    def track(self, roomba):
        print("Tracking: " + str(roomba))
        self.task_controller.switch_task(
            'TrackRoombaTask',
            target_roomba = roomba,
            offset_xy = [0, 0],
            timeout = 0
        )

    def update(self, delta, elapsed, environment):
        # switch every 8 seconds
        if (elapsed - self.last_switch > 8000):
            self.target = (self.target + 1) % cfg.MISSION_NUM_TARGETS

            # construct the new task
            self.go_to(self.target)

            # TODO: implement logging
            print("New target: " + str(self.target))

            self.last_switch = elapsed
