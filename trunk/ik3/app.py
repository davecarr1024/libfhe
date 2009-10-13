#!/usr/bin/env python

from time import time as systime
import math
import random

import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from robot import Robot
from target import Target
from segment import Segment
from joint import Joint
from vec3 import Vec3
import prims

class App:
    def __init__(self):
        pygame.init()
        glutInit(())
        self.screen = pygame.display.set_mode((800,600),
                                              pygame.OPENGL |
                                              pygame.DOUBLEBUF |
                                              pygame.HWSURFACE)

        glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_NORMALIZE)
        glClearColor(1,1,1,1)

        self.lastRenderTime = 0
        self.renderTime = 1/60.0

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45,800/600.0,1,1000)

        glMatrixMode(GL_MODELVIEW)

        self.target = Target(Vec3(1,1,1))
        self.lastTargetTime = 0
        self.targetTime = 2

        self.robot = Robot(Vec3.ZERO,
                           self.target,
                           Joint(Vec3.UNIT_Y,0,
                                 Segment(1,Joint(Vec3.UNIT_Z,math.pi/4,
                                                 Segment(5,
                                                         Joint(Vec3.UNIT_Z,0,
                                                               Segment(4,None)))))))

        self.camYaw = math.pi/2
        self.camHeight = 0
        self.camDist = 20
        self.camYawSpeed = 0
        self.camHeightSpeed = 0
        self.camDistSpeed = 0

    def tick(self, time, dtime):
        if time - self.lastRenderTime >= self.renderTime:
            self.lastRenderTime = time
            self.render()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.shutdown = True
            elif event.type == pygame.KEYDOWN:
                self.keydown(event)
            elif event.type == pygame.KEYUP:
                self.keyup(event)

        self.camYaw += self.camYawSpeed * dtime
        self.camHeight += self.camHeightSpeed * dtime
        self.camDist += self.camDistSpeed * dtime

        if time - self.lastTargetTime > self.targetTime:
            self.lastTargetTime = time
            self.target.pos = Vec3(random.uniform(-5,5),
                                   random.uniform(-5,5),
                                   random.uniform(-5,-5))

        self.robot.tick(time,dtime)

    def keydown(self, event):
        if event.key == pygame.K_LEFT:
            self.camYawSpeed = 1
        elif event.key == pygame.K_RIGHT:
            self.camYawSpeed = -1
        elif event.key == pygame.K_UP:
            self.camHeightSpeed = 10
        elif event.key == pygame.K_DOWN:
            self.camHeightSpeed = -10
        elif event.key == pygame.K_PAGEUP:
            self.camDistSpeed = -10
        elif event.key == pygame.K_PAGEDOWN:
            self.camDistSpeed = 10
        elif event.key == pygame.K_ESCAPE:
            self.shutdown = True

    def keyup(self, event):
        if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
            self.camYawSpeed = 0
        elif event.key == pygame.K_UP or event.key == pygame.K_DOWN:
            self.camHeightSpeed = 0
        elif event.key == pygame.K_PAGEDOWN or event.key == pygame.K_PAGEUP:
            self.camDistSpeed = 0

    def render(self):
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)

        glLoadIdentity()
        camx = self.camDist * math.cos(self.camYaw)
        camz = self.camDist * math.sin(self.camYaw)
        gluLookAt(camx,self.camHeight,camz,0,0,0,0,1,0)

        self.robot.render()

        glFlush()
        pygame.display.flip()

    def run(self):
        lastTime = start = systime()
        self.shutdown = False
        numFrames = 0

        while not self.shutdown:
            numFrames += 1
            time = systime()
            dtime = time - lastTime
            lastTime = time
            self.tick(time-start,dtime)

        print float(numFrames)/(systime()-start)

if __name__ == "__main__":
    App().run()
