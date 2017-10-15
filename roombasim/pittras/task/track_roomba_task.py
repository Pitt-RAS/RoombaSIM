
import numpy as np

from roombasim.ai import Task

class TrackRoombaTask(Task):

    def __init__(self, target_roomba):
        '''
        target - 3d (x,y,z) vector
        '''
        self.target_roomba = target_roomba

    def update(self, delta, elapsed, state_controller, environment):
        # fetch roomba odometry
        target_roombas, _ = state_controller.query('RoombaState', environment)

        # fetch drone odometry
        drone_state = state_controller.query('DroneState', environment)

        if not self.target_roomba in target_roombas:
            # self.complete(TaskState.FAILURE, "Roomba not found")
            return

        roomba = target_roombas[self.target_roomba]

        

