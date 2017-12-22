'''
land_demo_controller.py
'''

from roombasim.ai import Controller, TaskState

class LandDemoController(Controller):
    '''
    Demonstrates a takeoff followed by a landing.
    '''

    def land_callback(self, status, message):
        print("Landing callback")

        if status == TaskState.SUCCESS:
            print("Landing successful!")

    def setup(self):
        self.task_controller.switch_task(
            'TakeoffTask',
            callback=(lambda a,b: self.land())
        )

    def land(self):
        self.task_controller.switch_task(
            'LandTask',
            callback=self.land_callback
        )
