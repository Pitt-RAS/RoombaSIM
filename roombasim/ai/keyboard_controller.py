'''
keyboard_controller.py

A controller and task to allow for manual, keyboard control.

The KeyboardController will run only the KeyboardTask but
the task can be incorporated into other controllers to allow
for a switchover of manual control
'''

from pyglet.window import key

from .controller import Controller
from .task import Task, TaskController

class KeyboardController(Controller):
    '''
    Controller that only runs the keyboard task.
    '''
    def __init__(self, window=None):
        self.task_controller = TaskController({
            'keyboard': KeyboardTask
        })
        self.task_controller.switch_task('keyboard', window=window)
        self.state_controller = None

class KeyboardTask(Task):
    '''
    Simple keyboard flight controller:

    I/K +y acceleration
    L/J +x acceleration

    D/A +yaw velocity
    W/S +z velocity
    '''   
    def __init__(self, window=None):
        self.window = window
        self.keys = key.KeyStateHandler()
        self.window.push_handlers(self.keys)

    def update(self, delta, elapsed, state_controller, environment):
        x_accel = 0
        y_accel = 0
        yaw_vel = 0
        z_vel = 0

        if self.keys[key.J]:
            y_accel = 0.3
        elif self.keys[key.L]:
            y_accel = -0.3

        if self.keys[key.I]:
            x_accel = 0.3
        elif self.keys[key.K]:
            x_accel = -0.3

        if self.keys[key.A]:
            yaw_vel = 0.8
        elif self.keys[key.D]:
            yaw_vel = -0.8

        if self.keys[key.W]:
            z_vel = 0.3
        elif self.keys[key.S]:
            z_vel = -0.3

        environment.agent.control([x_accel, y_accel], yaw_vel, z_vel)
