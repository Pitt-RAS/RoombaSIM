'''
land_demo_controller.py
'''

from roombasim.ai import Controller
from roombasim.ai.task import TaskState

class LandDemoController(Controller):

    def setup(self):
        self._switched = False

        # TODO(mbilker): is there a way to initialize the drone to an
        # arbitrary height above PITTRAS_LAND_HEIGHT_TOLERANCE instead of using
        # TakeoffTask to get the height up
        self.task_controller.switch_task(
            'TakeoffTask'
        )

    def land_callback(self, status, message):
        print("Landing callback")

        if status == TaskState.SUCCESS:
            print("Landing successful!")

    def update(self, delta, elapsed):
        # switch to landing after five seconds
        if not self._switched and elapsed > 5000:
            print("Switching to LandTask")

            self._switched = True

            self.task_controller.switch_task(
                'LandTask',
                callback = self.land_callback
            )
