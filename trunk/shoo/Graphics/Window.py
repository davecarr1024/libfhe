import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

from Core.Vec2 import Vec2
from Core.OptionServer import optionServer
from Core.MessageServer import messageServer
from SceneNode2 import SceneNode2

class Window:
    def __init__(self):
        self.res = optionServer.get('res',Vec2(800,600))
        self.frameTime = 1.0 / optionServer.get('fps',60)
        self.fullscreen = optionServer.get('fullscreen',False)
        
        pygame.init()
        flags = pygame.OPENGL | pygame.DOUBLEBUF | pygame.HWSURFACE
        if self.fullscreen:
            flags |= pygame.FULLSCREEN
        self.screen = pygame.display.set_mode(tuple(self.res),flags)
        pygame.display.set_caption('Shoo')

        glEnable(GL_TEXTURE_2D)
        glShadeModel(GL_SMOOTH)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glLineWidth(1)
        glClearColor(1,1,1,1)

        self.lastFrameTime = None

        self.root2 = SceneNode2()

        messageServer.subscribe('update',self.update)

    def update(self, time, dtime):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                messageServer.publish('shutdown')
        
        if not self.lastFrameTime or time - self.lastFrameTime > self.frameTime:
            self.lastFrameTime = time

            glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)

            self.set2dProjection()
            self.root2.render()

            glFlush()
            pygame.display.flip()

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

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_NORMALIZE)

window = Window()
