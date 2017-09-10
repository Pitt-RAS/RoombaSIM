'''
roombasim.py

CLI interface for various functions.
'''

import pyglet
import argparse

from roombasim.display import Display
import roombasim.config as cfg
from roombasim.mission import Mission

def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command')

    demo_parser = subparsers.add_parser('demo')

    demo_parser.add_argument('-num_targets', type=int, choices=range(1,25))
    demo_parser.add_argument('-num_obstacles', type=int, choices=range(1,11))
    demo_parser.add_argument('-target_spawn_radius', type=float)
    demo_parser.add_argument('-obstacle_spawn_radius', type=float)

    args = parser.parse_args()

    if args.command == 'demo':
        run_demo(args)


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

    # setup mission
    m = Mission()
    m.setup()

    config = pyglet.gl.Config(sample_buffers=1, samples=4)
    window = Display(m)

    pyglet.app.run()


if __name__ == '__main__':
    main()