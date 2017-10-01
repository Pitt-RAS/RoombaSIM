'''
task.py

Contains TaskController and Task base classes
'''

class TaskController(object):
    '''
    A class that represents a sort of state machine that can
    switch between classes and update the currently running
    task via a single interface.
    '''
    def __init__(self, tasks):
        '''
        Initialize with a dictionary of tasks.

        Key/value pairs should be as follows:

        {
            'task_name': TaskClass
        }

        Note: don't supply an instance of a class, this will be
        created when switching to that task.
        '''
        self.tasks = tasks

        self.tasks['idle'] = IdleTask
        self.switch_task('idle')

    def switch_task(self, task_name, **params):
        '''
        Switch to a new task by name with optional params.

        Note: params are supplied as kwargs and will be given
        to the task constructor as kwargs.
        '''
        self.current = self.tasks[task_name](**params)

    def update(self, delta, elapsed, state_controller, environment):
        '''
        Perform an update step.
        '''
        if self.current:
            self.current.update(delta, elapsed, state_controller, environment)
        # else:
        #     print("[*] Warning: No task selected")

class Task(object):
    '''
    The base Task class.

    Subclasses must implement the update method and can optionally
    override the __init__ constructor.
    '''
    def __init__(self, **params):
        self.params = params
    
    def update(self, delta, elapsed, state_controller, environment):
        '''
        Perform an update step 
        '''
        raise NotImplementedError()


class IdleTask(object):
    '''
    A task that does nothing.
    '''
    def update(self, delta, elapsed, state_controller, environment):
        pass
        