'''
display.py

Contains a Display class which can be used to visualize
a mission round.

The graphics are entirely decoupled from the simulation
engine and this file contains all the implementations
of the drawing methods.

Graphics are drawn to the screen using pyglet as an
OpenGL interface. This should provide realtime-capable
graphics as well as relatively cross-platform availability.
'''
import pyglet
from pyglet.gl import *
import numpy as np
import time

import roombasim.config as cfg

from roombasim import roomba, geometry

class Display(pyglet.window.Window):

    def __init__(self, mission, self_update=True):
        super(Display, self).__init__(700,700)

        if self_update:
            pyglet.clock.schedule_interval(self._update, 1.0/60.0)
            pyglet.clock.set_fps_limit(60)

        self.mission = mission
        self.start_time = time.time()

    def _update(self, dt):
        self.mission.update(dt, (time.time() - self.start_time) * 1000)

    def on_resize(self, width, height):
        glViewport(10, 10, width-20, height-20)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, 20, 0, 20, -1, 1)

        # glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable( GL_BLEND );

    def on_draw(self):
        pyglet.clock.tick()
        glClear(GL_COLOR_BUFFER_BIT)

        Display._draw_gridlines()

        for r in self.mission.roombas:
            if isinstance(r, roomba.TargetRoomba):
                Display._draw_target_roomba(r)
            else:
                Display._draw_obstacle_roomba(r)

        Display._draw_drone(self.mission.agent)

    @staticmethod
    def _draw_target_roomba(r):
        pos = r.pos
        heading = r.heading
        vertex_count = cfg.GRAPHICS_CIRCLE_VERTICES
        radius = cfg.ROOMBA_RADIUS

        glEnable(GL_BLEND)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT,GL_NICEST)

        # Outline
        if r.state == cfg.ROOMBA_STATE_FORWARD or True:
            glColor3f(1,1,1)
        elif r.state == cfg.ROOMBA_STATE_TURNING:
            glColor3f(1,0.8,0.8)

        Display._draw_hollow_circle(pos, radius)

        # Direction indicator
        glBegin(GL_LINES)

        glVertex2f(pos[0], pos[1])
        glVertex2f(np.cos(heading) * radius + pos[0], np.sin(heading) * radius + pos[1])

        glEnd()

    @staticmethod
    def _draw_obstacle_roomba(r):
        pos = r.pos
        heading = r.heading
        vertex_count = cfg.GRAPHICS_CIRCLE_VERTICES
        radius = cfg.ROOMBA_RADIUS

        glEnable(GL_BLEND)
        glEnable(GL_LINE_SMOOTH)
        glHint(GL_LINE_SMOOTH_HINT,GL_NICEST)

        # Outline
        if r.state == cfg.ROOMBA_STATE_FORWARD or True:
            glColor3f(1,0.2,0.2)
        elif r.state == cfg.ROOMBA_STATE_TURNING:
            glColor3f(1,0.8,0.8)

        Display._draw_hollow_circle(pos, radius)

        # Direction indicator
        glBegin(GL_LINES)

        glVertex2f(pos[0], pos[1])
        glVertex2f(np.cos(heading) * radius + pos[0], np.sin(heading) * radius + pos[1])

        glEnd()
    
    @staticmethod
    def _draw_drone(drone):
        if cfg.RENDER_AGENT != None:
            cfg.RENDER_AGENT(drone)

    @staticmethod
    def _draw_gridlines():

        # draw horizontal lines
        for y in range(0,21):
            if y % 5 == 0:
                glColor3f(0.5,0.5,0.5)
            else:
                glColor3f(0.25,0.25,0.25)

            glBegin(GL_LINES)
            glVertex2f(0,y)
            glVertex2f(20,y)
            glEnd()

        # draw vertical lines
        for x in range(0,21):
            if x % 5 == 0:
                glColor3f(0.5,0.5,0.5)
            else:
                glColor3f(0.25,0.25,0.25)

            glBegin(GL_LINES)
            glVertex2f(x,0)
            glVertex2f(x,20)
            glEnd()

    @staticmethod
    def _draw_hollow_circle(pos, radius):
        glBegin(GL_LINE_LOOP)

        for i in range(cfg.GRAPHICS_CIRCLE_VERTICES):
            theta = (2 * np.pi * i) / cfg.GRAPHICS_CIRCLE_VERTICES
            glVertex2f(
                (np.cos(theta) * radius + pos[0]), 
                (np.sin(theta) * radius + pos[1])
            )

        glEnd()

    @staticmethod
    def _draw_hollow_square(pos, heading, diagonal):
        glBegin(GL_LINE_LOOP)
        # front right
        glVertex2f(
            pos[0] + (np.cos(heading - (cfg.PI / 4)) * diagonal),
            pos[1] + (np.sin(heading - (cfg.PI / 4)) * diagonal)
        )
        # back right
        glVertex2f(
            pos[0] + (np.cos(heading - (3 * cfg.PI / 4)) * diagonal),
            pos[1] + (np.sin(heading - (3 * cfg.PI / 4)) * diagonal)
        )
        # back left
        glVertex2f(
            pos[0] + (np.cos(heading + (3 * cfg.PI / 4)) * diagonal),
            pos[1] + (np.sin(heading + (3 * cfg.PI / 4)) * diagonal)
        )
        # front left
        glVertex2f(
            pos[0] + (np.cos(heading + (cfg.PI / 4)) * diagonal),
            pos[1] + (np.sin(heading + (cfg.PI / 4)) * diagonal)
        )
        glEnd()