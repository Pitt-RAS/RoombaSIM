'''
greediy_herding_v1.py
'''

import numpy as np

import roombasim.config as cfg

from roombasim.ai import Controller
import roombasim.geometry as geometry

class GreedyHerdingV1State(object):
    DONE = 0

    SEARCH = 1   # waiting to acquire target roomba
    
    # target init
    TARGET_INIT = 2     # choose to enter block or bump cycle

    # block cycle
    BLOCK_INIT = 3      # determine where to start blocking
    BLOCK_EXECUTE = 4   # block negative movement
    BLOCK_PREDICT = 5   # move to next predicted block location
    BLOCK_PREVENT = 6
    BLOCK_WAIT = 7

    # bump cycle
    BUMP_TRACK = 8      # track the target roomba
    BUMP_EXECUTE = 9    # execute top bump

class Logger(object):
    '''
    This is an example of a logger class.

    In the future, it would make sense to put this functionality in the core simulator.
    '''
    time = 0

    @staticmethod
    def log(state, message):
        print('%d\t[GreedyHerdingV1] :: %s :: %s' % (Logger.time, state, message))

    @staticmethod
    def set_time(t):
        Logger.time = t

# export the log function
log = Logger.log

class GreedyHerdingV1(Controller):
    '''
    General algorithm:

    --------------------------------------------------------------------------
    while there is a target roomba in the arena
        pick the one closest to the target wall -> r
        while r has not left the arena:
            if r is not perpendicular to the target wall:
                perform a top bump
            else if r is facing away from the target wall and there is time:
                immediately block r by landing in front of it
            else:
                wait until r initiates a reverse turn and immediately block it
    --------------------------------------------------------------------------  

    Planning is achieved by representing the AI as a finite state mahine where each state has
    access to the update() function and can also launch tasks in the TaskController. This method
    allows for relatively simple developement of complex looking behavior. There are some 
    '''

    def setup(self):
        self.state = None
        self.target_roomba = None
        self.tracker = None

        print('[GreedyHerdingV1] :: Takeoff!')

        self.task_controller.switch_task(
            'TakeoffTask',
            callback=self._switch_state_wrapper(GreedyHerdingV1State.SEARCH)
        )

    def _switch_state_wrapper(self, s):
        self.state = s

    def update(self, delta, elapsed, environment):

        # once the logger is built in we wouldn't need to do this
        Logger.set_time(elapsed)

        target_roombas, _ = self.state_controller.query('RoombaState', environment)

        if self.target_roomba is not None and self.tracker is not None:
            self.tracker.update(target_roombas[self.target_roomba], delta, elapsed)

            # if the roomba has left the arena, find a new target
            if is_outside_arena(target_roombas[self.target_roomba]) and self.state != GreedyHerdingV1State.DONE:
                log('<update>', 'Target roomba left arena, finding new target...')
                self.state = GreedyHerdingV1State.SEARCH

        if self.state == GreedyHerdingV1State.SEARCH:
            # TODO: (optimization) Instead of searching for the closest roomba to the wall, perhaps 
            # consider things like roomba heading and turn period to find the easiest roomba to remove.

            # find the roomba closest to the target wall
            remaining_roombas = {
                tag:r for tag,r in target_roombas.iteritems() 
                if not is_outside_arena(r) and tag != self.target_roomba
            }

            if len(remaining_roombas) == 0:
                log('SEARCH', 'we\'re done here')
                self.state = GreedyHerdingV1State.DONE
                return
            
            target = remaining_roombas.keys()[0]
            for tag in remaining_roombas:
                if target_roombas[tag]['pos'][0] > target_roombas[target]['pos'][0]:
                    target = tag

            # select target
            self.target_roomba = target

            # initialize tracker
            self.tracker = RoombaTracker()

            log('SEARCH', 'select target %s' % self.target_roomba)

            self.state = GreedyHerdingV1State.TARGET_INIT

        elif self.state == GreedyHerdingV1State.TARGET_INIT:
            # determine if roomba is aligned
            r = target_roombas[self.target_roomba]
            h = self.tracker.get_target_heading()

            # It's a bit hard to coordinate things while the roomba is turning, so we'll
            # just hover over it
            if self.tracker.is_reversing:
                if self.task_controller.current.__class__.__name__ != 'TrackRoombaTask':
                    self.task_controller.switch_task(
                        'TrackRoombaTask',
                        offset_xy=[0,0],
                        target_roomba=self.target_roomba,
                        timeout=0
                    )
                return

            if is_aligned(h):
                log('TARGET_INIT', 'Roomba is aligned')
                self.state = GreedyHerdingV1State.BLOCK_INIT
                return
            else:
                if self.task_controller.current.__class__.__name__ != 'GoToRoombaTask':
                    log('TARGET_INIT', 'Roomba is not aligned')
                    self.task_controller.switch_task(
                        'GoToRoombaTask',
                        target_roomba=self.target_roomba,
                        offset_xy=[0, 0],
                        callback=(lambda a,b: self._switch_state_wrapper(GreedyHerdingV1State.BUMP_TRACK))
                    )

        elif self.state == GreedyHerdingV1State.BLOCK_INIT:

            if will_leave_arena(target_roombas[self.target_roomba], self.tracker, elapsed):
                log('BLOCK_INIT', 'Roomba will leave, finding new target...')
                self.state = GreedyHerdingV1State.SEARCH
                return

            # check if the roomba has started reversing (either due to the drone blocking it
            # or the timer reaching zero) and find new task
            if self.tracker.is_reversing:
                self.state = GreedyHerdingV1State.TARGET_INIT
                return

            if self.task_controller.current.__class__.__name__ == 'GoToRoombaTask':
                pass
            else:
                # check if the roomba is facing (or will face) the target wall
                if geometry.compare_angle(self.tracker.get_target_heading(), 0) < (np.pi / 4):
                    log('BLOCK_INIT', 'Roomba is facing target')
                    self.state = GreedyHerdingV1State.BLOCK_PREDICT
                    return
                else:
                    log('BLOCK_INIT', 'Correcting roomba direction')

                    self.task_controller.switch_task(
                        'GoToRoombaTask',
                        target_roomba=self.target_roomba,
                        offset_xy=[1.5, 0],
                        callback=(lambda a,b: self._switch_state_wrapper(GreedyHerdingV1State.BLOCK_PREVENT))
                    ) 

        elif self.state == GreedyHerdingV1State.BLOCK_PREDICT:
            # continuously update prediction
            ((px, py), rem) = predict_block_point(target_roombas[self.target_roomba], self.tracker, elapsed)

            # if the roomba is about to turn, execute the block
            if rem < 2000:
                self.state = GreedyHerdingV1State.BLOCK_EXECUTE
                return

            # if the roomba has become unaligned, realign
            if not is_aligned(self.tracker.get_target_heading()):
                log('BLOCK_PREDICT', 'Realigning heading')
                self.state = GreedyHerdingV1State.BUMP_TRACK
                return

            # check if the roomba has started turning by itself
            if self.tracker.is_reversing:
                log('BLOCK_PREDICT', 'Roomba started reversing early, collision?')
                self.state = GreedyHerdingV1State.TARGET_INIT
                return

            if self.task_controller.current.__class__.__name__ != 'XYZTranslationTask':
                # launch the task
                log('BLOCK_PREDICT', 'Dynamic prediction at (%f, %f) timeout (%d)' % (px, py, rem))

                self.task_controller.switch_task(
                    'XYZTranslationTask',
                    target=[px,py,1],
                    hold_position=True,
                    normalize_yaw=True
                )
            else:
                # task exists, just update target
                self.task_controller.current.set_target([px, py, 1])

        elif self.state == GreedyHerdingV1State.BLOCK_EXECUTE:

            # block the roomba
            if self.task_controller.current.__class__.__name__ != 'BlockTask':

                h = target_roombas[self.target_roomba]['heading']

                self.task_controller.switch_task(
                    'BlockTask',
                    block_duration=4500,
                    target_roomba=self.target_roomba,
                    callback=(lambda a,b: self._switch_state_wrapper(GreedyHerdingV1State.TARGET_INIT))
                )

                log('BLOCK_EXECUTE', 'Blocking!')

        elif self.state == GreedyHerdingV1State.BLOCK_PREVENT:

            # check if the roomba has started reversing (either due to the drone blocking it
            # or the timer reaching zero) and find new task
            if self.tracker.is_reversing:
                self.state = GreedyHerdingV1State.TARGET_INIT
                return

            # immediate block
            if self.task_controller.current.__class__.__name__ != 'BlockRoombaTask':
                log('BLOCK_PREVENT', 'Blocking!')

                self.task_controller.switch_task(
                    'BlockRoombaTask',
                    target_roomba=self.target_roomba,
                    block_vector=[1.5, 0],
                    callback=(lambda a,b: self._switch_state_wrapper(GreedyHerdingV1State.TARGET_INIT))
                )

        elif self.state == GreedyHerdingV1State.BUMP_TRACK:

            # check if we should bump
            if should_bump(target_roombas[self.target_roomba], self.tracker):
                if self.task_controller.current.__class__.__name__ != 'TrackRoombaTask':
                    # track the roomba then bump

                    # the drone has a hard time reaching the roomba sometimes so we will
                    # set an offset a bit in front of the roomba
                    d = 0.3

                    h = target_roombas[self.target_roomba]['heading']
                    ox = np.cos(h) * d
                    oy = np.sin(h) * d

                    self.task_controller.switch_task(
                        'TrackRoombaTask',
                        offset_xy=[ox, oy],
                        target_roomba=self.target_roomba,
                        timeout=1500,
                        callback=(lambda a,b: self._switch_state_wrapper(GreedyHerdingV1State.BUMP_EXECUTE))
                    )

            else:
                self.state = GreedyHerdingV1State.TARGET_INIT
                return

        elif self.state == GreedyHerdingV1State.BUMP_EXECUTE:
            if self.task_controller.current.__class__.__name__ != 'HitRoombaTask':
                # bump the roomba
                log('BUMP_EXECUTE', 'Bumping roomba!')

                self.task_controller.switch_task(
                    'HitRoombaTask',
                    target_roomba=self.target_roomba,
                    callback=(lambda a,b: self._switch_state_wrapper(GreedyHerdingV1State.BUMP_TRACK))
                )


class RoombaTracker(object):
    '''
    A class to track a target roomba's position and predict certain internal states
    of the roomba.
    '''
    def __init__(self):
        # TODO: (improvement) implement some sort of Kalman filter to track roomba position/velocity
        # in order to handle noisy sensors
        self._last_pos = np.array([0, 0])

        # reverse takes approximately three seconds but this value may need to be modified
        self._reverse_duration = 3000   # ms

        # the last heading of the roomba when it was moving linearly
        self.linear_heading = 0

        # true if the roomba is not moving linearly and recently started reversing
        self.is_reversing = False

    def update(self, roomba_state, delta, elapsed):
        '''
        Update
        '''
        # TODO: (improvement) this is shitty tracking but it works when given perfect knowledge
        self.pred_vel = (roomba_state['pos'] - self._last_pos) / delta
        self._last_pos = np.array(roomba_state['pos'])

        speed = np.linalg.norm(self.pred_vel)

        # if the roomba is moving linearly, update the current heading
        if speed > 0.15:
            self.linear_heading = roomba_state['heading']

        # if the roomba has stopped and recently started reversing
        self.is_reversing = (speed < 0.15) and (elapsed - roomba_state['reverse_timer'] < self._reverse_duration)

    def get_target_heading(self):
        '''
        Returns the current heading if the roomba is moving linearly or the target heading
        after a reverse period if the roomba is currently turning.
        '''
        target_heading = self.linear_heading

        if self.is_reversing:
            target_heading += np.pi

        return target_heading


def is_aligned(heading):
    '''
    Determine if the given roomba is aligned with the target wall
    '''
    return (geometry.compare_angle(heading, 0) < np.pi/4) or (geometry.compare_angle(heading, np.pi) < np.pi/4)


def is_facing_wall(heading):
    '''
    Returns true if the roomba is aligned and facing the target wall
    '''
    return (geometry.compare_angle(heading, 0) < np.pi/4)


def predict_block_point(roomba_state, roomba_tracker, elapsed):
    '''
    Predicts when and where to block a roomba's next 180 degree turn.

    Currently, this method assumes the roomba will travel in a straight line for the
    remainder of the reverse period.

    Returns ((x,y), rem) where (x,y) is the position and rem is the time remaining in
    milliseconds before the full turn.
    '''
    pos = roomba_state['pos']
    h = roomba_tracker.get_target_heading()
    t = cfg.ROOMBA_REVERSE_PERIOD - (elapsed - roomba_state['reverse_timer'])

    px = pos[0] + (np.cos(h) * cfg.ROOMBA_LINEAR_SPEED * t / 1000.0)
    py = pos[1] + (np.sin(h) * cfg.ROOMBA_LINEAR_SPEED * t / 1000.0)

    # TODO: optimize this
    block_dist = 1

    bx = px - (np.cos(h) * block_dist)
    by = py - (np.sin(h) * block_dist)

    return ((bx, by), t)


def will_leave_arena(roomba_state, roomba_tracker, elapsed):
    '''
    This method tries to predict whether a roomba will leave the arena within the current
    reverse period.

    It uses a naive approach of testing whether the next predicted block point is at least
    1 meter past the arena boundary.

    TODO: (improvement) generate a probability of the roomba leaving based on how many
    noisy turns it has left to make, how far it is from the wall and how far it is from
    either of the adjacent walls and use that to make a "safer" prediction

    TODO: (improvement) check if there are other roombas in the way that would prevent this
    one from leaving the arena (this may require more advanced AI features such as internally
    simulating the game state which would likely also help in long-term pathfinding)
    '''
    ((x,y), t) = predict_block_point(roomba_state, roomba_tracker, elapsed)

    return x > 20.5


def is_outside_arena(roomba_state):
    '''
    Returns true if the given roomba is outside the arena.
    '''
    [x, y] = roomba_state['pos']
    return (x < 0 or x > 20 or y < 0 or y > 20)


def should_bump(roomba_state, roomba_tracker):
    '''
    Returns true if the roomba should be top-bumped in order to align the heading
    with the target wall.
    '''
    heading = roomba_tracker.linear_heading

    correction = min(
        geometry.compare_angle(0, heading),
        geometry.compare_angle(np.pi, heading)
    )

    # correct if the roomba is more than 30 degrees misaligned
    return correction > np.deg2rad(30)
