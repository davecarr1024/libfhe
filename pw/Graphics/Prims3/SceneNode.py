# -*- coding: utf-8 -*-

from Core.Math.SpatialNode3 import SpatialNode3
from Graphics.MaterialManager import materialManager
from Graphics.Window import Window

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
    
    def msg_render3(self):
        self.transform()
        materialManager.bind(self.getVar("material",{}))
    
        #if self.getVar("static",False) and not self.listNode:
            #if self.list:
                #glCallList(self.list)
                #return
            #else:
                #self.list = glGenLists(1)
                #glNewList(self.list,GL_COMPILE_AND_EXECUTE)
                #self.listNode = self
        self.geom()
            
    def unmsg_render3(self, **args):
        self.untransform()
        materialManager.unbind()
        
        #if self.listNode == self:
            #glEndList()
            #self.listNode = None
            
    def getWindow(self):
        window = self.searchAncestors(lambda node: isinstance(node,Window))
        assert window
        return window
