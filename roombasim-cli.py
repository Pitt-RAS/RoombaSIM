#!/usr/bin/env python
'''
roombasim.py

CLI interface for various functions.
'''

import argparse
import time
import pyglet

from roombasim.graphics.display import Display
import roombasim.config as cfg
from roombasim.environment import Environment

def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command')

    demo_parser = subparsers.add_parser('demo')
    demo_parser.add_argument('-num_targets', type=int, choices=range(1,25))
    demo_parser.add_argument('-num_obstacles', type=int, choices=range(1,11))
    demo_parser.add_argument('-target_spawn_radius', type=float)
    demo_parser.add_argument('-obstacle_spawn_radius', type=float)

    speedtest_parser = subparsers.add_parser('speedtest')
    speedtest_parser.add_argument('-frames', type=int, default=1000)

    args = parser.parse_args()

    if args.command == 'demo':
        run_demo(args)
    elif args.command == 'speedtest':
        speed_test(args)


def run_demo(args):
    '''
    Runs a visual demo of roomba movement
    '''
    if args.num_targets != None:
        cfg.MISSION_NUM_TARGETS = args.num_targets

    if args.num_obstacles != None:
        cfg.MISSION_NUM_OBSTACLES = args.num_obstacles

    if args.target_spawn_radius != None:
        cfg.MISSION_TARGET_SPAWN_RADIUS = args.target_spawn_radius

    if args.obstacle_spawn_radius != None:
        cfg.MISSION_OBSTACLE_SPAWN_RADIUS = args.obstacle_spawn_radius


    import roombasim.pittras.config
    cfg.load(roombasim.pittras.config)

    # setup mission
    e = Environment()
    e.reset()

    # setup agent
    d = cfg.AGENT([13,10], 0)
    e.agent = d

    config = pyglet.gl.Config(sample_buffers=1, samples=4)
    window = Display(e)

    pyglet.app.run()


def speed_test(args):
    import roombasim.pittras.config
    cfg.load(roombasim.pittras.config)

    n = args.frames

    print 'Starting speed test [{} frames]'.format(n)

    e = Environment()
    e.reset()

    d = cfg.AGENT([13,10], 0)
    e.agent = d

    start = time.time()

    i = 0
    el = 0
    while i < n:
        el += 1/60.
        e.update(1/60., el)
        i += 1

    end = time.time()

    dur = end - start
    mul = n / dur

    print 'Processing {} frames took {} seconds'.format(n, dur)
    print 'Speed of {} fps'.format(mul)

if __name__ == '__main__':
    main()