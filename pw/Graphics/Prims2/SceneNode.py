# -*- coding: utf-8 -*-

from Core.SpatialNode2 import SpatialNode2

from OpenGL.GL import *

class SceneNode(SpatialNode2):
    listNode = None
    
    def onAttach(self):
        SpatialNode2.onAttach(self)
        self.list = None
    
    def transform(self):
        glPushMatrix()
        m = self.getVar("globalTransform")
        glMultMatrixf(m[0],m[1],m[2],0, m[3],m[4],m[5],0, m[6],m[7],m[8],0, 0,0,0,1)
        
    def untransform(self):
        glPopMatrix()
        
    def geom(self):
        pass
    
    def msg_render2(self, **args):
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

    def unmsg_render2(self, **args):
        if self.listNode == self:
            glEndList()
            self.listNode = None
        
        self.untransform()
