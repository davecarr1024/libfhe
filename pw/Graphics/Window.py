import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from Core.Node import Node
from Core import Util

class Window(Node):
    def onAttach(self):
        self.inputFuncs = Util.getFuncs(self,'input_')
        
        pygame.init()
        
        res = self.getVar("res",(800,600))
        flags = pygame.OPENGL | pygame.DOUBLEBUF | pygame.HWSURFACE
        if self.getVar("fullscreen",False):
            flags |= pygame.FULLSCREEN
        self.screen = pygame.display.set_mode(res, flags)
        
        glClearColor(*self.getVar("clearColor",(1,1,1,1)))
        
        self.initKeys()
        
    def initKeys(self):
        self.keyNames = dict([(getattr(pygame,name),name[2:].lower()) for name in dir(pygame) if name.startswith('K_')])
        self.keyMods = dict([(getattr(pygame,name),name[5:].lower()) for name in dir(pygame) if name.startswith('KMOD_')])

    def msg_update(self, time, dtime):
        for event in pygame.event.get():
            name = pygame.event.event_name(event.type).lower()
            if name in self.inputFuncs:
                self.inputFuncs[name](event)
            else:
                self.log("unhandled event",event)
        
        if time - self.getVar("lastRenderTime",0) > 1.0 / self.getVar("fps",60):
            self.setVar("lastRenderTime",time)
            self.render()

    def render(self):
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
        
        self.publish("render")
        
        glFlush()
        pygame.display.flip()
        
    def input_quit(self, event):
        self.globalPublish("shutdown")
        
    def input_keydown(self, event):
        key = self.keyNames.get(event.key,None)
        if key:
            self.publish('keyDown_%s' % key)

    def input_keyup(self, event):
        key = self.keyNames.get(event.key,None)
        if key:
            self.publish('keyUp_%s' % key)
            
    def msg_keyDown_escape(self):
        self.globalPublish("shutdown")
