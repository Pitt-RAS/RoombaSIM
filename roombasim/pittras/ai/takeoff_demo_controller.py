'''
takeoff_demo_controller.py
'''
from roombasim.ai import Controller

class TakeoffDemoController(Controller):
    '''
    Demonstrates a takeoff
    '''

    def setup(self):
        self.task_controller.switch_task(
            'TakeoffTask'
        )
