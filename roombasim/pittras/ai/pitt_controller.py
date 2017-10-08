'''
pitt_controller.py
'''

from roombasim.ai import Controller

class PittController(Controller):

    def setup(self):
        self.task_controller.switch_task(
            'XYZTranslateTask',
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

    def update(self, delta, elapsed):
        if (elapsed - self.last_switch > 5000):
            self.selected = (self.selected + 1) % len(self.waypoints)
            waypoint = self.waypoints[self.selected]

            self.task_controller.switch_task(
                'XYZTranslateTask',
                target = waypoint
            )

            print "New target:", waypoint

            self.last_switch = elapsed
