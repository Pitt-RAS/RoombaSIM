'''
waypoint_demo_controller.py
'''

from roombasim.ai import Controller

class WaypointDemoController(Controller):
    '''
    A demo controller that uses the XYZTranslationTask to move
    the drone to four waypoints in a circular motion.
    '''

    def setup(self):
        # set the initial target
        self.task_controller.switch_task(
            'XYZTranslationTask',
            target = [5,5,2]
        )

        self.last_switch = 0

        self.waypoints = [
            [5,5,2],
            [15,5,1],
            [15,15,2],
            [5,15,1]
        ]

        self.selected = 0

    def update(self, delta, elapsed, environment):
        # switch after five seconds
        if (elapsed - self.last_switch > 5000):
            self.selected = (self.selected + 1) % len(self.waypoints)
            waypoint = self.waypoints[self.selected]

            # construct the new task
            self.task_controller.switch_task(
                'XYZTranslationTask',
                target = waypoint
            )

            # TODO: implement logging
            print("New target: " + str(waypoint))

            self.last_switch = elapsed
