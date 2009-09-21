# -*- coding: utf-8 -*-
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from Core.Node import Node
from Core import Util
from Core.Math.Vec2 import Vec2
from Core.Math.SpatialNode2 import SpatialNode2 

class Window(Node):
    def onAttach(self):
        self.initGraphics()
        self.initInput()
        
    def initInput(self):
        self.keyNames = {}
        self.keyMods = {}
        for name in dir(pygame):
            if name.startswith('K_'):
                self.keyNames[getattr(pygame,name)] = name[2:].lower()
            elif name.startswith('KMOD_'):
                self.keyMods[name[5:].lower()] = getattr(pygame,name)

        self.joystickCount = pygame.joystick.get_count()
        self.joysticks = []
        for i in range(self.joystickCount):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)

        self.mouseButtonNames = {1:'left',2:'middle',3:'right'}

    def initGraphics(self):
        pygame.init()
        glutInit(())
        flags = pygame.OPENGL | pygame.DOUBLEBUF | pygame.HWSURFACE
        if self.getVar("fullscreen",False):
            flags |= pygame.FULLSCREEN
            
        self.res = self.getVar("res",(100,100))
        self.frameTime = 1.0 / self.getVar("fps",60)
        self.screen = pygame.display.set_mode(self.res,flags)
        pygame.display.set_caption(self.getVar("title","pw"))
        
        glEnable(GL_TEXTURE_2D)
        glShadeModel(GL_SMOOTH)
        
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glLineWidth(self.getVar("lineWidth",1))
        glClearColor(*self.getVar("clearColor",(1,1,1,1)))
        
        self.lastFrameTime = None
        fps = self.getVar("fps",0)
        if fps:
            self.frameTime = 1.0 / fps
        else:
            frameTime = 0
        
    def onDetach(self):
        pygame.quit()
        
    def msg_update(self, time, dtime):
        if not self.lastFrameTime: self.lastFrameTime = time
        
        if time - self.lastFrameTime > self.frameTime:
            self.lastFrameTime += self.frameTime
            
            glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
            
            self.set3dProjection()
            self.publish("render3")
            self.set2dProjection()
            self.publish("render2")
            
            glFlush()
            pygame.display.flip()
            
        for event in pygame.event.get():
            eventName = pygame.event.event_name(event.type)
            eventName = eventName[0].lower() + eventName[1:]
            self.call("input_%s" % eventName, event)
                
    def input_quit(self, event):
        self.globalPublish('shutdown')

    def input_keyDown(self, event):
        if event.key == pygame.K_ESCAPE:
            self.globalPublish('shutdown')
        else:
            name, mods = self.keyInfo(event)
            self.publish('keyDown', name = name, mods = mods, key = event.key)

    def input_keyUp(self, event):
        name, mods = self.keyInfo(event)
        self.publish('keyUp', name = name, mods = mods, key = event.key)

    def input_joyButtonDown(self, event):
        self.publish('joyButtonDown',joy = event.joy, button = event.button)

    def input_joyButtonUp(self, event):
        self.publish('joyButtonUp',joy = event.joy, button = event.button)

    def input_joyAxisMotion(self, event):
        self.publish('joyAxisMotion',joy = event.joy, value = event.value, axis = event.axis)

    def input_mouseButtonDown(self, event):
        buttonName, pos, absPos = self.mouseInfo(event)
        hitObjects = self.doPicking(pos)
        self.publish('mouseButtonDown',button = event.button, pos = pos, absPos = absPos, buttonName = buttonName, hitObjects = hitObjects)

    def input_mouseButtonUp(self, event):
        buttonName, pos, absPos = self.mouseInfo(event)
        hitObjects = self.doPicking(pos)
        self.publish('mouseButtonUp', button = event.button, pos = pos, absPos = absPos, buttonName = buttonName, hitObjects = hitObjects)

    def input_mouseMotion(self, event):
        pos, absPos = self.mousePos(event)
        self.setVar("mousePos",pos)
        buttonNames = [self.mouseButtonNames.get(i) for i in event.buttons]
        self.publish('mouseMotion', pos = pos, absPos = absPos, buttons = event.buttons, buttonNames = buttonNames)
        
    def keyInfo(self, event):
        name = self.keyNames.get(event.key)

        mods = []
        for modName, mod in self.keyMods.iteritems():
            if mod & event.mod:
                mods.append(modName)

        return name, mods

    def mousePos(self, event):
        absPos = Vec2(*event.pos)
        pos = absPos / Vec2(*map(float,self.res))
        return pos, absPos

    def mouseInfo(self, event):
        pos, absPos = self.mousePos(event)
        return self.mouseButtonNames.get(event.button), pos, absPos

    def doPicking(self, pos):
        children = self.filterDescendents(lambda node: isinstance(node,SpatialNode2))
        return sum([child.collide(pos) for child in children],[])
        
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

