'''
state.py

Contains StateController and State base classes
'''

class StateController(object):
    '''
    Contains a collection of state subclasses and can perform
    queries by state name.
    '''
    def __init__(self, states):
        '''
        Initialize with a state dictionary.
        '''
        self.states = states

    def query(self, state_name, environment):
        '''
        Perform a sensor lookup by name.
        '''
        if state_name in self.states:
            return self.states[state_name].query(environment)
        else:
            return None # uh oh
        

class State(object):
    '''
    Subclasses must override the query method.
    '''
    @staticmethod
    def query(environment):
        '''
        Takes a reference to the environment and returns some data.
        '''
        raise NotImplementedError()
        