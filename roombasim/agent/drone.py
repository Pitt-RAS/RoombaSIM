'''
drone.py

Generic Drone implementation
'''
import numpy as np
import roombasim.config as cfg

class Drone(object):
    '''
    Extend this generic class to provide team
    specific implementation details.

    =========================
    Simplified drone physics:
    =========================

    To control 2d position relative to the origin, the
    drone can control xy_accel and yaw_vel:

    - xy_accel is a 2d vector corresponding to the 2d 
    acceleration vector in the drone's reference frame. So 
    a vector of [0, 1] would correspond to the drone accelerating 
    in 2d space at a rate of 1 m/s^2 in the direction of its
    current yaw. This vector is rotated about the origin
    by the yaw in order to calculate xy_vel and xy_pos which
    are both in the global reference frame.

    - yaw_vel is a value corresponding to the current angular
    velocity of the drone. It is asssumed, therefore that the 
    drone has infinite angular acceleration and can set an
    instataneous angular velocity.

    To control altitude, the drone can control z_vel:

    - z_vel is a value corresponding to the 1d velocity
    parameter in the z direction. In this way, the z axis of
    motion is completely disconected from the xy axes which
    makes simulating motion quite a bit simpler.
    '''

    def __init__(self, pos, yaw, z_pos=0):
        # 2d position vector
        # m
        self.xy_pos = np.array(pos, dtype=np.float64)

        # 2d velocity vector (with respect to origin)
        # m/s
        self.xy_vel = np.array([0,0], dtype=np.float64)

        # 2d acceleration vector (with respect to current heading)
        # m/s^2
        self.xy_accel = np.array([0,0], dtype=np.float64)

        # 2d acceleration vector (with respect to origin)
        #
        # this is created by rotating the acceleration vector
        # about the orign by the current yaw
        self._frame_accel = np.array([0,0], dtype=np.float64)

        # yaw direction
        # rad
        self.yaw = yaw

        # angular velocity
        # rad/s
        self.yaw_vel = 0

        # distance from the ground
        # (when landed, this should be zero)
        # m
        self.z_pos = z_pos

        # z axis velocity (+ is up)
        # m/s
        self.z_vel = 0

    def control(self, xy_accel, yaw_vel, z_vel):
        '''
        Update drone target parameters:

        - xy_accel : a 2d array containing target x and y acceleration values
        - yaw_vel : a value containing the target yaw angular velocity
        - z_vel : a value containing the target z velocity
        '''
        self.xy_accel = np.array(xy_accel, dtype=np.float64)

        # Make sure acceleration is within drone limits
        if np.linalg.norm(self.xy_accel) > cfg.DRONE_MAX_HORIZ_ACCEL:
            self.xy_accel *= (cfg.DRONE_MAX_HORIZ_ACCEL
                            / np.linalg.norm(self.xy_accel))

        self.yaw_vel = yaw_vel

        self.z_vel = z_vel

        # Make sure z velocity is within drone limits
        if np.abs(self.z_vel) > cfg.DRONE_MAX_VERTICAL_VELOCITY:
            self.z_vel = np.copysign(cfg.DRONE_MAX_VERTICAL_VELOCITY,
                                     self.z_vel)

    def update(self, delta, elapsed):
        '''
        Perform a physics update step.
        '''
        # update height
        self.z_pos += self.z_vel * delta

        # bounds check
        if self.z_pos <= 0:
            self.z_pos = 0
            self.z_vel = 0
            self.xy_vel = np.array((0.0, 0.0))
        else:
            # update yaw
            self.yaw += self.yaw_vel * delta
            self.yaw %= (np.pi * 2)

            # update position

            # rotate the acceleration vector about the origin by
            # the current yaw and apply it to the velocity vector
            rot_matrix = np.array([
                [np.cos(self.yaw), -np.sin(self.yaw)],
                [np.sin(self.yaw), np.cos(self.yaw)]
            ])

            self._frame_accel = rot_matrix.dot(self.xy_accel)

            self.xy_vel += self._frame_accel * delta

            # Make sure drone velocity is within limits
            if np.linalg.norm(self.xy_vel) > cfg.DRONE_MAX_HORIZ_VELOCITY:
                self.xy_vel *= (cfg.DRONE_MAX_HORIZ_VELOCITY
                              / np.linalg.norm(self.xy_vel))

            self.xy_pos += self.xy_vel * delta

    # The following functions should be implemented by
    # a team-specific subclass:

    def is_touching_roomba_top(self, rba):
        '''
        True if the drone is making contact with roomba top switch.
        '''
        raise NotImplementedError

    def is_blocking_roomba(self, rba):
        '''
        True if drone is making contact with roomba front bumper.
        '''
        raise NotImplementedError
