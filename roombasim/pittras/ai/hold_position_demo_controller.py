'''
hold_position_demo_controller.py
'''

from roombasim.ai import Controller

class TakeoffDemoController(Controller):
    '''
    A demo controller tests HoldPositionTask.
    '''

    def setup(self):
        self.task_controller.switch_task(
            'HoldPositionTask',
            hold_duration = 5
        )
