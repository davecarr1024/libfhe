# -*- coding: utf-8 -*-

from Core.Node import Node

import pygame

from OpenGL.GL import *
from OpenGL.GLU import *

class Material(Node):
    textures = {}
    
    def msg_render2(self):
        self.bind()
        
    def unmsg_render2(self):
        self.unbind()
        
    def msg_render3(self):
        self.bind()
        
    def msg_unrender3(self):
        self.unbind()
        
    @classmethod
    def loadTexture(self, filename):
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

    @classmethod
    def bindTexture(self, filename):
        if filename not in self.textures:
            self.textures[filename] = self.loadTexture(filename)
        glBindTexture(GL_TEXTURE_2D,self.textures[filename])

    @classmethod
    def unbindTexture(self):
        glBindTexture(GL_TEXTURE_2D,0)

    def bind(self):
        glPushAttrib(GL_CURRENT_BIT | GL_LIGHTING_BIT)
        glMatrixMode(GL_TEXTURE)
        glPushMatrix()
        
        for func in filter(lambda f: f.startswith("bind_"),self.funcs.iterkeys()):
            prop = f[len("bind_")+1:]
            if self.hasVar(prop):
                self.call(func,self.getVar(prop))
                
        glMatrixMode(GL_MODELVIEW)
        
    def unbind(self):
        glPopAttrib()
        glMatrixMode(GL_TEXTURE)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        
    def bind_texture(self, filename):
        self.bindTexture(filename)

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
