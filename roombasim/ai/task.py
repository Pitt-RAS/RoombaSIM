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

    def switch_task(self, task_name, callback=None, **params):
        '''
        Switch to a new task by name with optional params.

        If the callback parameter is specified, once the task
        completes, it will be called with the following signature:

        >>> callback(status, message)

        Note: params are supplied as kwargs and will be given
        to the task constructor as kwargs.
        '''
        self.current = self.tasks[task_name](**params)
        self.current.set_completion_callback(self.callback_wrapper(callback))

    def callback_wrapper(self, callback):
        '''
        Creates a callback function of signature (status, method)
        that will set the current task to 'idle' and will optionally
        call an additional callback function if it exists.
        '''
        if callback is not None:
            def fn(status, message):
                self.switch_task('idle')
                callback(status, message)
            
            return fn
        else:
            def fn(status, message):
                self.switch_task('idle')

            return fn

    def update(self, delta, elapsed, state_controller, environment):
        '''
        Perform an update step.
        '''
        if self.current:
            self.current.update(delta, elapsed, state_controller, environment)
        # else:
        #     print("[*] Warning: No task selected")

class TaskState(object):
    '''
    An enum for task completion status.
    '''
    SUCCESS = 0
    FAILURE = 1

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

    def set_completion_callback(self, fn):
        self._completion_callback = fn

    def complete(self, status, message=''):
        '''
        Signal the end of the task.

        - status : TaskState enum
        - [message] : optional message string
        '''
        if hasattr(self, '_completion_callback'):
            self._completion_callback(status, message)
        

class IdleTask(Task):
    '''
    A task that does nothing.
    '''
    def update(self, delta, elapsed, state_controller, environment):
        pass

