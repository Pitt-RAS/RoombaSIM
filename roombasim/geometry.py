'''
geometry.py

Utility methods for working in 2D space.
'''

import numpy as np

import roombasim.config as cfg

PI = cfg.PI
TAU = cfg.TAU
ROOT2 = np.sqrt(2)

def circle_intersects_circle(a_center, b_center, radius):
    '''
    Returns True if the two circles intersect.

    a_center, b_center - the center of each circle
    radius - the radius of each circle
    '''
    dist = pow(a_center[0] - b_center[0], 2) + pow(a_center[1] - b_center[1], 2)
    intersects = dist < (4 * radius * radius)
    return intersects

def compare_angle(a, b):
    '''
    Returns the smallest absolute difference between the 
    two angles in radians.

    a, b - two angles to compare in radians
    '''
    return abs((((a - b) + PI) % TAU) - PI)

def circle_intersects_square(c_center, c_radius, s_center, s_heading, s_width):
    '''
    Returns True if the given circle intersects the square.

    c_center - center of the circle
    c_radius - radius of the circle

    s_center - center of the square
    s_heading - angle of the square perpendicular to a side
    s_width - width of the sides of the square
    '''
    corners = get_square_corners(s_center, s_heading, s_width)

    intersects = False
    intersects |= circle_intersects_line(c_center, c_radius, corners[0], corners[1])
    intersects |= circle_intersects_line(c_center, c_radius, corners[1], corners[2])
    intersects |= circle_intersects_line(c_center, c_radius, corners[2], corners[3])
    intersects |= circle_intersects_line(c_center, c_radius, corners[3], corners[0])
    
    return intersects

def get_square_corners(center, heading, width):
    '''
    Returns a list of the four corners of the given square.

    center - center of the square
    heading - angle of the square perpendicular to a side
    width - width of the sides of the square
    '''
    diagonal = width / ROOT2

    corners = [
        [
            center[0] + (np.cos(heading - (PI / 4)) * diagonal),
            center[1] + (np.sin(heading - (PI / 4)) * diagonal)
        ],
        [
            center[0] + (np.cos(heading - (3 * PI / 4)) * diagonal),
            center[1] + (np.sin(heading - (3 * PI / 4)) * diagonal)
        ],
        [
            center[0] + (np.cos(heading + (3 * PI / 4)) * diagonal),
            center[1] + (np.sin(heading + (3 * PI / 4)) * diagonal)
        ],
        [
            center[0] + (np.cos(heading + (PI / 4)) * diagonal),
            center[1] + (np.sin(heading + (PI / 4)) * diagonal)
        ]
    ]

    return corners

def circle_intersects_line(circle, radius, p0, p1):
    '''
    Returns True if the line passes through the circle.

    circle - center of circle
    radius - radius of circle
    p0, p1 - endpoints of line
    '''
    # black magic
    
    segment = np.array([p1[0]-p0[0], p1[1]-p0[1]])
    seg_circ = np.array([circle[0]-p0[0], circle[1]-p0[1]])

    norm_seg = segment / np.linalg.norm(segment)

    dot = np.dot(norm_seg, seg_circ)
    project = norm_seg * np.dot(norm_seg, seg_circ)

    if dot < 0 or dot > np.linalg.norm(segment):
        # projected vector is in the wrong direction
        # or larger than the original segment vector
        return False

    proj_point = project + np.array(p0)

    dist = np.linalg.norm(proj_point - np.array(circle))

    return dist < radius

def rotate_vector(vector, theta):
    '''
    Returns the given vector rotated clockwise about the origin
    by theta.
    '''
    rot_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    return rot_matrix.dot(vector)
