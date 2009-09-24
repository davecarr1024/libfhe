# -*- coding: utf-8 -*-

from Core.Math.SpatialNode2 import SpatialNode2
from Graphics.MaterialManager import materialManager
from Graphics.Window import Window

from OpenGL.GL import *

class SceneNode(SpatialNode2):
    listNode = None
    
    def onAttach(self):
        SpatialNode2.onAttach(self)
        self.list = None
    
    def transform(self):
        SpatialNode2.msg_render2(self)
        #glPushMatrix()
        #m = self.getVar("globalTransform")
        #self.log("transform",m)
        #glLoadMatrixf((m[0],m[1],m[2],0, 
                       #m[3],m[4],m[5],0, 
                       #m[6],m[7],m[8],0, 
                       #0,0,0,1))
        
    def untransform(self):
        SpatialNode2.unmsg_render2(self)
        #glPopMatrix()
        
    def geom(self):
        pass
    
    def msg_render2(self):
        self.transform()

        self.calledList = False
        if self.getVar("static",True) and not SceneNode.listNode:
            if self.list:
                self.calledList = True
                glCallList(self.list)
                return
            else:
                self.list = glGenLists(1)
                glNewList(self.list,GL_COMPILE_AND_EXECUTE)
                SceneNode.listNode = self

        materialManager.bind(self.getVar("material",{}))
        self.geom()
        
    def unmsg_render2(self, **args):
        self.untransform()

        if self.calledList: return
        
        materialManager.unbind()

        if SceneNode.listNode == self:
            glEndList()
            SceneNode.listNode = None
            
    def getWindow(self):
        window = self.searchAncestors(lambda node: isinstance(node,Window))
        assert window
        return window
