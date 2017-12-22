'''
controller.py

Contains the base controller implementation.
'''
import roombasim.config as cfg

from .task import TaskController
from .state import StateController

class Controller(object):
    '''
    Base controller class.

    Subclases should implement the update method.
    '''
    def __init__(self):
        self.task_controller = TaskController(cfg.TASKS)
        self.state_controller = StateController(cfg.STATES)
        self.setup()

    def frame_update(self, delta, elapsed, environment):
        '''
        This method should be called by the update loop in order to
        update the controller as well as propogate task updates to
        the environment.
        '''
        self.update(delta, elapsed, environment)
        self.task_controller.update(delta, elapsed, self.state_controller, environment)

    def setup(self):
        '''
        Optional post-initialization setup method
        '''
        pass

    def update(self, delta, elapsed, environment):
        '''
        Subclasses should override this method to perform AI decision making.
        '''
        pass
