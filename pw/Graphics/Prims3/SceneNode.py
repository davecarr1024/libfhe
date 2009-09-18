# -*- coding: utf-8 -*-

from Core.SpatialNode3 import SpatialNode3

from OpenGL.GL import *

class SceneNode(SpatialNode3):
    listNode = None
    
    def onAttach(self):
        SpatialNode3.onAttach(self)
        self.list = None
    
    def transform(self):
        glPushMatrix()
        glMultMatrixf(*self.getVar("globalTransform"))
        
    def untransform(self):
        glPopMatrix()
        
    def geom(self):
        pass
    
    def msg_render3(self, **args):
        self.transform()
    
        picking = args.get('picking')

        if picking:
            args['pickObjects'].append(self)
            glPushName(len(args['pickObjects']))
            
        doGeom = True

        if not picking and self.getVar("static",False) and not self.listNode:
            if self.list:
                glCallList(self.list)
                doGeom = False
            else:
                self.list = glGenLists(1)
                glNewList(self.list,GL_COMPILE_AND_EXECUTE)
                self.listNode = self

        if doGeom:
            self.geom()
        
        if picking:
            glPopName()

    def unmsg_render3(self, **args):
        if self.listNode == self:
            glEndList()
            self.listNode = None
        
        self.untransform()
