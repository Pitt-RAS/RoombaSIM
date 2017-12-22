
from roombasim.ai import Controller

import roombasim.config as cfg

class HitRoombaDemoController(Controller):

    def setup(self):
        self.takeoff(1)

    def takeoff(self, target):
        self.task_controller.switch_task(
            'TakeoffTask',
            callback=(lambda a,b: self.go_to_roomba(target))
        )

    def go_to_roomba(self, target):
        self.task_controller.switch_task(
            'GoToRoombaTask',
            target_roomba=target,
            offset_xy=[0,0],
            callback=(lambda a,b: self.hit_roomba(target))
        )

    def hit_roomba(self, target):
        self.task_controller.switch_task(
            'HitRoombaTask',
            target_roomba=target,
            callback=(lambda a,b: self.recover())
        )

    def recover(self):
        self.task_controller.switch_task(
            'XYZTranslationTask',
            target=[10,10,1],
            callback=(lambda a,b: self.hover())
        )

    def hover(self):
        self.task_controller.switch_task(
            'HoldPositionTask',
            hold_duration=0
        )
