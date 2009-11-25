from Box2D import *
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from time import time as systime
import math

class Shoo:
    def __init__(self):
        self.initPhysics()
        self.initGraphics()
        self.initInput()
        self.shutdown = False

        self.vehicles = []

    def initInput(self):
        self.joystickCount = pygame.joystick.get_count()
        self.joysticks = []
        for i in range(self.joystickCount):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)

    def initGraphics(self):
        pygame.init()

        self.res = 800,600

        self.screen = pygame.display.set_mode(self.res,pygame.OPENGL | pygame.DOUBLEBUF | pygame.HWSURFACE)

        glEnable(GL_TEXTURE_2D)
        glShadeModel(GL_SMOOTH)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glLineWidth(1)
        glClearColor(1,1,1,1)

        self.lastGraphicsUpdate = 0
        self.graphicsFrameTime = 1.0 / 60.0

    def updateGraphics(self, time, dtime):
        if time - self.lastGraphicsUpdate > self.graphicsFrameTime:
            self.lastGraphicsUpdate = time

            glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)

            self.set3dProjection()
            self.render3d()
            self.set2dProjection()
            self.render2d()

            glFlush()
            pygame.display.flip()

    def updateInput(self, time, dtime):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.shutdown = True

            elif event.type == pygame.JOYAXISMOTION:
                self.input('joy%d_axis%d' % (event.joy,event.axis),event.value,event.value > 0.5)

    def input(self, name, v, b):
        for vehicle in self.vehicles:
            vehicle.input(name,v,b)
        
    def render3d(self):
        p = self.cameraPos
        l = self.cameraLookAt
        gluLookAt(p.x,p.y,p.z,l.x,l.y,l.z,0,1,0)

        for vehicle in self.vehicles:
            vehicle.render()

    def render2d(self):
        pass

    def set2dProjection(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluOrtho2D(0,self.res[0],0,self.res[1])

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslate(0,self.res[1],0)
        glScalef(self.res[0],-self.res[1],1)

        glColor(1,1,1,1)
        glBindTexture(GL_TEXTURE_2D,0)

        #glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_NORMALIZE)

    def set3dProjection(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45,float(self.res[0])/float(self.res[1]),1,1000)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glColor(1,1,1,1)
        glBindTexture(GL_TEXTURE_2D,0)
        glBindTexture(GL_TEXTURE_3D,0)

        #glEnable(GL_LIGHTING)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_NORMALIZE)

    def initPhysics(self):
        worldBounds = b2AABB()
        worldBounds.lowerBound = (-100,-100)
        worldBounds.upperBound = (100,100)

        gravity = b2Vec2(0,-10)

        doSleep = True

        self.world = b2World(worldBounds,gravity,doSleep)

        groundBodyDef = b2BodyDef()
        groundBodyDef.position = (0,-10)

        groundBody = self.world.CreateBody(groundBodyDef)

        groundShapeDef = b2PolygonDef()
        groundShapeDef.SetAsBox(50,10)
        groundBody.CreateShape(groundShapeDef)

        self.lastPhysicsUpdate = 0
        self.physicsFrameTime = 0.01

    def updatePhysics(self, time, dtime):
        if time - self.lastPhysicsUpdate > self.physicsFrameTime:
            self.lastPhysicsUpdate += self.physicsFrameTime

            self.world.Step(self.physicsFrameTime,10,10)

            for vehicle in self.vehicles:
                vehicle.update(time,self.physicsFrameTime)

    def update(self, time, dtime):
        self.updateInput(time,dtime)
        self.updatePhysics(time,dtime)
        self.updateGraphics(time,dtime)

    def run(self):
        self.initGame()
        startTime = systime()
        lastTime = time = 0
        while not self.shutdown:
            time = systime() - startTime
            dtime = time - lastTime
            lastTime = time
            self.update(time,dtime)

    def initGame(self):
        from Vec3 import Vec3
        from Vehicle import Vehicle

        self.cameraPos = Vec3(0,10,-20)
        self.cameraLookAt = Vec3(0,0,0)

        self.vehicles.append(Vehicle((0,10)))

shoo = Shoo()

import Prims
from Vec3 import Vec3
from Vehicle import Vehicle
