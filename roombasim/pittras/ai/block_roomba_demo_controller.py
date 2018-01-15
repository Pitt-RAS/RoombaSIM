from roombasim.ai import Controller

import roombasim.config as cfg

class BlockRoombaDemoController(Controller):

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
            callback=(lambda a,b: self.block_roomba(target))
        )

    def block_roomba(self, target):
        self.task_controller.switch_task(
            'BlockRoombaTask',
            target_roomba=target,
            block_vector=(1.5, 0.0),
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
