# -*- coding: utf-8 -*-

from Math.Vec3 import Vec3
from Math.Quat import Quat
from Math.Mat4 import Mat4
from Math.Box3 import Box3
from Node import Node

from OpenGL.GL import *

class SpatialNode3(Node):
    def __init__(self, **args):
        Node.__init__(self, **args)
        
        self.localTransformValid = False
        self.globalTransformValid = False
        self.globalBoxValid = False
        
        def transform():
            if not self.localTransformValid:
                self.localTransformValid = True
                self.localTransform = Mat4.translation(self.getVar("pos",Vec3())) * Mat4.rotation(self.getVar("rot",Quat()))
            return self.localTransform
            
        self.setVar("transform",transform)
        
        def globalTransform():
            parent = self.getParentNode()
            if not parent:
                return self.getVar("transform")
                
            if not self.allAncestorsGlobalTransformValid():
                self.globalTransformValid = True
                self.globalTransform = parent.getVar("globalTransform") * self.getVar("transform")
                
            return self.globalTransform
                
        self.setVar("globalTransform",globalTransform)
        
        def globalBox():
            if not self.globalBoxValid:
                self.globalBox = self.getVar("box",Box3())
                for child in self.getChildNodes():
                    self.globalBox.expand(child.getVar("globalBox").transform(child.getVar("transform")))
            return self.globalBox
            
        self.setVar("globalBox",globalBox)
        
    def set_pos(self, pos):
        self.globalTransformValid = self.localTransformValid = False
        
    def set_rot(self, rot):
        self.globalTransformValid = self.localTransformValid = False
        
    def set_box(self, box):
        for node in self.getAncestors():
            node.globalBoxValid = False
        
    def onAttach(self):
        self.globalTransformValid = False
        
    def onDetach(self):
        self.globalTransformValid = False
        
    def getParentNode(self):
        return self.searchAncestors(lambda node: isinstance(node,SpatialNode3))
        
    def getAncestors(self):
        return self.filterAncestors(lambda node: isinstance(node,SpatialNode3), True)
        
    def getChildNodes(self):
        return self.filterDescendents(lambda node: isinstance(node,SpatialNode3))
        
    def allAncestorsGlobalTransformValid(self):
        return all([node.globalTransformValid for node in self.getAncestors()])
        
    def getRelTransform(self, node):
        return self.getVar("globalTransform").inverse() * node.getVar("globalTransform")
        
    def overlaps(self, node):
        return self.getVar("globalBox").overlaps(node.getVar("globalBox").transform(self.getRelTransform(node)))
        
    def msg_render3(self, **args):
        glPushMatrix()
        self.getVar("pos",Vec3()).glTranslate()
        self.getVar("rot",Quat()).glRotate()
        
    def unmsg_render3(self, **args):
        glPopMatrix()

if __name__ == "__main__":
    import math
    
    n1 = SpatialNode3(vars = dict(box = Box3(Vec3(-1,-1,-1),Vec3(1,1,1))))
    n2 = SpatialNode3(vars = dict(box = Box3(Vec3(-1,-1,-1),Vec3(1,1,1))))
    
    assert n1.overlaps(n2)
    
    n2.setVar("pos",Vec3(10,10,10))
    assert not n1.overlaps(n2)

    n3 = SpatialNode3(parent = n1, vars = dict(box = Box3(Vec3(-1,-1,-1),Vec3(1,1,1)), pos = Vec3(5,5,5)))
    assert not n3.overlaps(n2)
    
    n1.setVar("pos",Vec3(5,5,5))
    assert n3.overlaps(n2)
    
    n1.setVar("rot",Quat.fromAxisAngle(Vec3.UNIT_X,math.pi/2))
    assert not n3.overlaps(n2)
