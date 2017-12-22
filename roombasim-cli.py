#!/usr/bin/env python
'''
roombasim.py

CLI interface for various functions.
'''

import argparse
import time
import pyglet

import roombasim.config as cfg
from roombasim.graphics import Display
from roombasim.environment import Environment


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command')

    controller_parser = subparsers.add_parser('run')
    controller_parser.add_argument('config')
    controller_parser.add_argument('controller')
    controller_parser.add_argument('-start_location',
                                   type=float,
                                   nargs=3,
                                   default=[1.5, 1.5, 0.0])

    demo_parser = subparsers.add_parser('demo')
    demo_parser.add_argument('-num_targets', type=int, choices=range(1,25))
    demo_parser.add_argument('-num_obstacles', type=int, choices=range(1,11))
    demo_parser.add_argument('-target_spawn_radius', type=float)
    demo_parser.add_argument('-obstacle_spawn_radius', type=float)

    speedtest_parser = subparsers.add_parser('speedtest')
    speedtest_parser.add_argument('-frames', type=int, default=1000)

    keydemo_parser = subparsers.add_parser('keydemo')

    args = parser.parse_args()

    if args.command == 'run':
        run_controller(args)
    elif args.command == 'demo':
        run_demo(args)
    elif args.command == 'speedtest':
        speed_test(args)
    elif args.command == 'keydemo':
        keyboard_demo(args)


def _load_class(cpath):
    '''
    Attempts to load a class given the module path.
    Returns the class reference or None if it was not found.
    '''
    try:
        attr = cpath.split('.')
        mod = __import__('.'.join(attr[:-1]))
        for a in attr[1:]:
            mod = getattr(mod, a)
        return (mod, None)
    except Exception as e:
        return (None, e)


def run_controller(args):
    config, err = _load_class(args.config)

    if (config is None):
        print("Couldn't load config path: " + str(args.config))
        print('See the following error:\n')
        print(err)
        return
    else:
        cfg.load(config)

    controller_p, err = _load_class(args.controller)

    if (controller_p is None):
        print("Couldn't load class: " + str(args.controller))
        print('See the following error:\n')
        print(err)
        return
    else:
        print("Loaded Controller: " + str(controller_p))
    
    # initialize controller
    controller = controller_p()

    # setup mission
    environment = Environment()
    environment.reset()

    # setup agent
    location = args.start_location
    agent = cfg.AGENT([location[0],location[1]], 0, location[2])
    environment.agent = agent

    # create window so the keyboard can access it
    window = Display(environment)

    def update_func(delta, elapsed):
        environment.update(delta, elapsed)
        controller.frame_update(delta, elapsed, environment)

    window.set_update_func(update_func)
    config = pyglet.gl.Config(sample_buffers=1, samples=4)

    pyglet.app.run()


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
    environment = Environment()
    environment.reset()

    # setup agent
    agent = cfg.AGENT([13,10], 0)
    environment.agent = agent

    config = pyglet.gl.Config(sample_buffers=1, samples=4)
    window = Display(environment)

    def update_func(delta, elapsed):
        environment.update(delta, elapsed)

    window.set_update_func(update_func)

    pyglet.app.run()


def keyboard_demo(args):
    import roombasim.pittras.config
    from roombasim.ai import KeyboardController, KeyboardTask

    cfg.load(roombasim.pittras.config)

    # setup mission
    environment = Environment()
    environment.reset()

    # setup agent
    agent = cfg.AGENT([1.5,1.5], 0)
    environment.agent = agent

    # create window so the keyboard can access it
    window = Display(environment)

    # setup controller
    controller = KeyboardController(window=window)

    def update_func(delta, elapsed):
        environment.update(delta, elapsed)
        controller.frame_update(delta, elapsed, environment)

    window.set_update_func(update_func)
    config = pyglet.gl.Config(sample_buffers=1, samples=4)

    pyglet.app.run()


def speed_test(args):
    import roombasim.pittras.config
    cfg.load(roombasim.pittras.config)

    n = args.frames

    print('Starting speed test [{} frames]'.format(n))

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

    print('Processing {} frames took {} seconds'.format(n, dur))
    print('Speed of {} fps'.format(mul))


if __name__ == '__main__':
    main()

