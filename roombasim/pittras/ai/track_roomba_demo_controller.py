'''
track_roomba_demo_controller.py
'''

from roombasim.ai import Controller

import roombasim.config as cfg

class TrackRoombaDemoController(Controller):
    '''
    
    '''

    def setup(self):
        # initial target
        self.target = 0

        # set the initial target
        self.task_controller.switch_task(
            'GoToRoombaTask',
            target_roomba = self.target,
            offset_xy = [-1, 0]
        )

        self.last_switch = 0

    def update(self, delta, elapsed):
        # switch after five seconds
        if (elapsed - self.last_switch > 25000):
            self.target = (self.target + 1) % cfg.MISSION_NUM_TARGETS

            # construct the new task
            self.task_controller.switch_task(
                'GoToRoombaTask',
                target_roomba = self.target,
                offset_xy = [-1, 0]
            )

            # TODO: implement logging
            print("New target: " + str(self.target))

            self.last_switch = elapsed
