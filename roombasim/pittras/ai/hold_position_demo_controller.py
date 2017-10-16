'''
hold_position_demo_controller.py
'''

from roombasim.ai import Controller

def hold_position_task_completion_callback(status):
    '''
    Callback for Hold Position task completion.
    '''
    print("Hold Position task completed with", status)

class HoldPositionDemoController(Controller):
    '''
    A demo controller tests HoldPositionTask.
    '''

    def setup(self):
        self.task_controller.switch_task(
            'HoldPositionTask',
            callback=hold_position_task_completion_callback,
            hold_duration=5
        )
