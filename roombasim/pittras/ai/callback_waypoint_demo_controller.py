'''
callback_waypoint_demo_controller.py
'''

from roombasim.ai import Controller

class CallbackWaypointDemoController(Controller):
    '''
    A demo controller that uses the XYZTranslationTask to move
    the drone to four waypoints in a circular motion.
    '''

    def setup(self):
        # set the initial target
        self.task_controller.switch_task(
            'XYZTranslationTask',
            callback = self.waypoint_callback,
            target = [5,5,2]
        )

        self.waypoints = [
            [5,5,2],
            [7.5,3,1],
            [17,5,1],
            [17,6,1],
            [19,19,2],
            [15,15,2],
            [5,15,1]
        ]

        self.selected = 0

    def waypoint_callback(self, status, message):
        print("Waypoint reached!")
        self.set_new_waypoint()

    def set_new_waypoint(self):
        self.selected = (self.selected + 1) % len(self.waypoints)
        waypoint = self.waypoints[self.selected]

        # construct the new task
        self.task_controller.switch_task(
            'XYZTranslationTask',
            callback = self.waypoint_callback,
            target = waypoint
        )

        print("New target: " + str(waypoint))
