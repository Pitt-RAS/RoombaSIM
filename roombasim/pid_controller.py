'''
PID Controller for AI tasks
'''

import numpy as np

class PIDController(object):
    '''
    Represents a PID controller for an arbitrary amount of dimensions. Initialize
    the controller with the three constants and use the `get_control` method to
    update the internal state and get the target vector.
    '''

    def __init__(self, k_pdi, dimensions=1):
        '''
        k_pdi should be a numpy array with the constants:
        [k_p, k_d, k_i]
        '''
        self.k_pdi = k_pdi
        self.i_error = np.zeros(dimensions, dtype=np.float64)
        self.last_p = np.zeros(dimensions, dtype=np.float64)

    def get_control(self, p_error, d_error, delta):
        '''
        Update the PID controller with a new error reading
        and time delta.

        p_error and d_error should be numpy arrays

        Returns the calculated control vector.
        '''
        self.i_error += p_error * delta
        control = self.k_pdi.dot([p_error, d_error, self.i_error])
        return control
