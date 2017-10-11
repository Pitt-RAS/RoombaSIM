
from roombasim.ai import Controller

class TakeoffDemoController(Controller):

    def setup(self):
        self.task_controller.switch_task(
            'TakeoffTask'
        )
    