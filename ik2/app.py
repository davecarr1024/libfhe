#!/usr/bin/env python

from time import time as systime

import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import math
import random

from vec2 import Vec2
from robot import Robot
from joint import Joint
from segment import Segment
from target import Target

class App:
    def __init__(self):
        pygame.init()
        glutInit(())
        pygame.display.set_mode((800,600),
                                pygame.OPENGL |
                                pygame.DOUBLEBUF |
                                pygame.HWSURFACE)

        glShadeModel(GL_SMOOTH)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glLineWidth(2)
        glClearColor(1,1,1,1)
        glDisable(GL_DEPTH_TEST)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluOrtho2D(0,800,0,600)

        glMatrixMode(GL_MODELVIEW)

        self.renderTime = 1/60.0
        self.lastRenderTime = 0

        self.target = Target(Vec2())

        self.moveTime = 1
        self.lastMoveTime = 0

        self.robot = Robot(self.target,
                           Vec2(400,300),
                           Joint(0,Segment(200,
                                           Joint(math.radians(45),
                                                 Segment(150,
                                                         Joint(math.radians(45),
                                                                  Segment(100,None)))))))

    def render(self):
        glLoadIdentity()
        glTranslate(0,600,0)
        glScalef(1,-1,1)

        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)

        self.robot.render()

        glFlush()
        pygame.display.flip()

    def tick(self, time, dtime):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.shutdown = True
            elif event.type == pygame.MOUSEMOTION:
                self.target.pos = Vec2(event.pos[0],event.pos[1])
                self.lastMoveTime = time

        if time - self.lastMoveTime > self.moveTime:
            self.lastMoveTime = time
            self.target.pos = Vec2(random.uniform(0,800),random.uniform(0,600))

        if time - self.lastRenderTime > self.renderTime:
            self.lastRenderTime = time
            self.render()

        self.robot.tick(time,dtime)

    def run(self):
        start = lastTime = systime()
        self.shutdown = False
        numFrames = 0
        while not self.shutdown:
            numFrames += 1
            time = systime()
            dtime = time - lastTime
            lastTime = time
            self.tick(time-start,dtime)
        print float(numFrames)/(systime()-start)

if __name__ == '__main__':
    App().run()
