# -*- coding: utf-8 -*-

from Math.Vec3 import Vec3
from Math.Quat import Quat
from Math.Mat4 import Mat4
from Math.Box3 import Box3
from Node import Node

class SpatialNode3(Node):
    def __init__(self, **args):
        Node.__init__(self, **args)
        
        def transform():
            return Mat4.translation(self.getVar("pos",Vec3())) * Mat4.rotation(self.getVar("rot",Quat()))
            
        self.setVar("transform",transform)
        
        def globalTransform():
            parent = self.getParentNode()
            if parent:
                return parent.getVar("globalTransform") * self.getVar("transform")
            else:
                return self.getVar("transform")
                
        self.setVar("globalTransform",globalTransform)
        
        def globalBox():
            box = self.getVar("box",Box3())
            for child in self.getChildNodes():
                box.expand(child.getVar("globalBox").transform(child.getVar("transform")))
            return box
            
        self.setVar("globalBox",globalBox)
        
    def getParentNode(self):
        return self.searchAncestors(lambda node: isinstance(node,SpatialNode3))
        
    def getChildNodes(self):
        return self.filterDescendents(lambda node: isinstance(node,SpatialNode3))
        
    def getRelTransform(self, node):
        return self.getVar("globalTransform").inverse() * node.getVar("globalTransform")
        
    def overlaps(self, node):
        return self.getVar("globalBox").overlaps(node.getVar("globalBox").transform(self.getRelTransform(node)))
        
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
