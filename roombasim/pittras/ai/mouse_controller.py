'''
mouse_controller.py
'''

from roombasim.ai import Controller
from roombasim.environment.roomba import TargetRoomba

class MouseController(Controller):
    '''
    A controller that lets the user control the drone with the mouse
    '''

    def setup(self):
        self.task_id = 0
        self.task_controller.switch_task('TakeoffTask')

    def mouse_callback(self, obj_clicked, button):
        self.task_id += 1
        if isinstance(obj_clicked, TargetRoomba):
            if button == 'left':
                self.task_controller.switch_task(
                    'GoToRoombaTask',
                    target_roomba=obj_clicked.tag,
                    offset_xy=(0.0, 0.0),
                    callback=self.task_callback(
                        lambda : self.task_controller.switch_task(
                            'HitRoombaTask',
                            target_roomba=obj_clicked.tag,
                            callback=self.task_callback(
                                lambda : self.task_controller.switch_task(
                                    'TakeoffTask',
                                    callback=self.task_callback(
                                        lambda : self.task_controller.switch_task(
                                            'HoldPositionTask',
                                            hold_duration=0)))))))
            else:
                self.task_controller.switch_task(
                    'GoToRoombaTask',
                    target_roomba=obj_clicked.tag,
                    offset_xy=(0.0, 0.0),
                    callback=self.task_callback(
                        lambda : self.task_controller.switch_task(
                            'BlockRoombaTask',
                            target_roomba=obj_clicked.tag,
                            block_vector=(1.5, 0.0),
                            callback=self.task_callback(
                                lambda : self.task_controller.switch_task(
                                    'TakeoffTask',
                                    callback=self.task_callback(
                                        lambda : self.task_controller.switch_task(
                                            'HoldPositionTask',
                                            hold_duration=0)))))))
        else:
            self.task_controller.switch_task(
                    'XYZTranslationTask',
                    target=(obj_clicked[0], obj_clicked[1], 1.0),
                    callback=self.task_callback(
                        lambda : self.task_controller.switch_task(
                            'HoldPositionTask',
                            hold_duration=0)))

    def task_callback(self, callback):
        def f(a, b, task_id=self.task_id):
            if self.task_id == task_id:
                callback()
        return f
