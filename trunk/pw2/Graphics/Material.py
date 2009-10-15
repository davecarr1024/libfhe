# -*- coding: utf-8 -*-

from Core.Aspect import Aspect

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame

class Material(Aspect):
    textures = {}
    
    def msg_render2(self):
        self.bind()
        
    def msg_render3(self):
        self.bind()
        
    def bind(self):
        glPushAttrib(GL_TEXTURE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT)
        glMatrixMode(GL_TEXTURE)
        glPushMatrix()
        
        for name in self.entity.getVarNames():
            self.tryCall("bind_%s" % name, None, self.getVar(name))
            
        glMatrixMode(GL_MODELVIEW)
        
    def unmsg_render2(self):
        self.unbind()
        
    def unmsg_render3(self):
        self.unbind()
        
    def unbind(self):
        glPopAttrib()
        glMatrixMode(GL_TEXTURE)
        glPopMatrix()
        
        for name in self.entity.getVarNames():
            self.tryCall("unbind_%s" % name, self.getVar(name))
        
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

    def bind_texture(self, filename):                                  
        Material.bindTexture(filename)

    def bind_color(self, c):
        glColor(*c)         

    def bind_ambient(self, c):
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,c)

    def bind_diffuse(self, c):
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,c)

    def bind_specular(self, c):
        glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,c)

    def bind_emission(self, c):
        glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,c)

    def bind_shininess(self, f):
        glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,f)

    def bind_translate(self, v):
        v.glTranslate()

    def bind_scale(self, v):
        v.glScale()

    def bind_rotate(self, r):
        r.glRotate()

    def bind_transform(self, m):
        glMultMatrixf(*m)
