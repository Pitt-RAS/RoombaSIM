
from roombasim.ai import Controller

class LandDemoController(Controller):

    def setup(self):
        # TODO(mbilker): initialize the drone to an arbitrary height above
        # PITTRAS_LAND_HEIGHT_TOLERANCE

        self.task_controller.switch_task(
            'LandTask'
        )
