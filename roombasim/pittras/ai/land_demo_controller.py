'''
land_demo_controller.py
'''

from roombasim.ai import Controller, TaskState

class LandDemoController(Controller):

    def setup(self):
        self._switched = False

        # TODO: initialize the drone to an arbitrary height because this is a
        # demo controller for testing LandTask.

        self.task_controller.switch_task(
            'LandTask'
        )

    def land_callback(self, status, message):
        print("Landing callback")

        if status == TaskState.SUCCESS:
            print("Landing successful!")

    def update(self, delta, elapsed):
        pass
