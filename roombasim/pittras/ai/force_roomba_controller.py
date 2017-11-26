'''
Test controller to follow a single roomba and force it
out of the arena to score a point.

Basic algorithm is as follows:

1. fly to the roomba
2. Perform a series of top bumps until the roomba is facing the correct wall
   or is facing slightly counterclockwise.
3. Track behind the roomba and orient bumper to be perpendicular to roomba motion.
4. Once the roomba starts turning (zero lateral motion) land the drone and wait
   for the roomba to make contact.
5. Repeat
'''

import numpy as np

from roombasim.ai import Controller
import roombasim.config as cfg
from roombasim import geometry

class ForceRoombaController(Controller):

    class _STATE(object):
         # flying to roomba
        init = 0

        # correcting roomba heading
        correcting = 1

        # tracking roomba
        tracking = 2

        # blocking roomba
        blocking = 3

    def setup(self):
        self.target_roomba = 7
        self._state = ForceRoombaController._STATE.init

        # keep track of roomba params
        self.target_heading = 0
        self.target_last_pos = np.array([0, 0], dtype=np.float64)
        self.target_last_vel = np.array([0, 0], dtype=np.float64)
        self.target_vel = [0, 0]

        # where to block roomba
        self.block_vector = np.array([0, 0], dtype=np.float64)

        print('Initiating takeoff...')
        self.task_controller.switch_task(
            'TakeoffTask',
            callback=(lambda a,b: self.init())
        )

    def init(self):
        '''
        Fly to the target roomba
        '''
        print('Flying to target: ' + str(self.target_roomba))
        self._state = ForceRoombaController._STATE.init

        self.task_controller.switch_task(
            'GoToRoombaTask',
            target_roomba=self.target_roomba,
            offset_xy=[0,0],
            callback=(lambda a,b: self.correct())
        )

    def correct(self):
        '''
        Correct the roomba heading until it is facing the correct wall
        '''
        self._state = ForceRoombaController._STATE.correcting

        def correct_a():
            '''
            Determine the heading of the roomba and decide whether to
            initiate a top bump or not.
            '''
            diff = geometry.compare_angle(self.target_heading, -cfg.PI / 8)

            if (diff < cfg.PI / 8):
                correct_b()
            else:
                self.track()

        def correct_b():
            '''
            Bump the roomba
            '''
            print('\t- Bump!')
            self.task_controller.switch_task(
                'HitRoombaTask',
                target_roomba=self.target_roomba,
                callback=(lambda a,b: correct_c())
            )

        def correct_c():
            '''
            Height recovery and then loop back to check heading
            '''
            print('\t- Height recovery')
            self.task_controller.switch_task(
                'TrackRoombaTask',
                target_roomba=self.target_roomba,
                offset_xy=[0,0],
                timeout=500,
                callback=(lambda a,b: correct_a())
            )

        print('Correcting roomba heading')
        correct_a()

    def track(self):
        print("Tracking roomba")
        self._state = ForceRoombaController._STATE.tracking

        self.task_controller.switch_task(
            'TrackRoombaTask',
            target_roomba=self.target_roomba,
            offset_xy=[0,0],
            timeout=0
        )

    def block(self):
        print("Blocking roomba")
        self._state = ForceRoombaController._STATE.blocking
        self.task_controller.switch_task(
            'BlockRoombaTask',
            target_roomba=self.target_roomba,
            block_vector=self.block_vector
        )


    def update(self, delta, elapsed, environment):
        target_roombas, _ = self.state_controller.query('RoombaState', environment)
        roomba_state = target_roombas[self.target_roomba]

        self.target_last_vel[:] = self.target_vel

        self.target_heading = roomba_state['heading']

        if not self.target_last_pos is None:
            self.target_vel = np.subtract(roomba_state['pos'], self.target_last_pos) / delta

        self.target_last_pos[:] = roomba_state['pos']

        # check if the roomba is about to do a full turn
        if self._state == ForceRoombaController._STATE.tracking and np.linalg.norm(self.target_vel) < 0.1:
            vel = np.linalg.norm(self.target_last_vel)
            d_vec = np.multiply(-1, self.target_last_vel) / vel if vel > 0 else 0
            self.block_vector = 1.5 * d_vec
            self.block()

        elif self._state == ForceRoombaController._STATE.blocking and np.linalg.norm(self.target_vel) > 0.1:
            self.track()
        
        




