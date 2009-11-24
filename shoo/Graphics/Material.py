from OpenGL.GL import *
from OpenGL.GLU import *
import pygame

class Material:
    textures = {}

    def __init__(self, **args):
        self.args = args

    def bind(self):
        glPushAttrib(GL_TEXTURE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT)
        glMatrixMode(GL_TEXTURE)
        glPushMatrix()

        if 'texture' in self.args:
            Material.bindTexture(self.args['texture'])

        if 'color' in self.args:
            glColor(*self.args['color'])

        if 'pos' in self.args:
            self.args['translate'].glTranslate()

        if 'rot' in self.args:
            self.args['rot'].glRotate()

        if 'scale' in self.args:
            self.args['scale'].glScale()

        glMatrixMode(GL_MODELVIEW)

    def unbind(self):
        glPopAttrib()
        glMatrixMode(GL_TEXTURE)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

    @staticmethod
    def loadTexture(filename):
        texture = pygame.image.load(fileServer.getFile(filename))
        w,h = texture.get_size()                                 
        textureData = pygame.image.tostring(texture,"RGBA",0)    
        textureId = glGenTextures(1)                             
        glBindTexture(GL_TEXTURE_2D,textureId)                   
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT)    
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT)    
        try:                                                          
            gluBuild2DMipmaps(GL_TEXTURE_2D,GL_RGBA,w,h,GL_RGBA,GL_UNSIGNED_BYTE,textureData)
        except OpenGL.error.NullFunctionError:                                               
            glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,w,h,0,GL_RGBA,GL_UNSIGNED_BYTE,textureData) 
        glBindTexture(GL_TEXTURE_2D,0)                                                       
        return textureId                                                                     

    @staticmethod
    def bindTexture(filename):
        if filename not in Material.textures:
            Material.textures[filename] = Material.loadTexture(filename)
        glBindTexture(GL_TEXTURE_2D,Material.textures[filename])    

    @staticmethod
    def unbindTexture():
        glBindTexture(GL_TEXTURE_2D,0)

