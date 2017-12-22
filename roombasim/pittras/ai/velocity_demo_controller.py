'''
velocity_demo_controller.py
'''

from roombasim.ai import Controller

class VelocityDemoController(Controller):
    '''
    A demo controller tests VelocityTask.
    '''

    def setup(self):
        self.task_controller.switch_task(
            'VelocityTask',
            callback=self.completion_callback,
            target=[2, 1, 0.2]
        )

    def completion_callback(self, status, message):
        '''
        Callback for Velocity task completion.
        '''
        print("Velocity task completed with", status)
