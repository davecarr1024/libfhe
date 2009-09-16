# -*- coding: utf-8 -*-

from Math.Vec2 import Vec2
from Math.Mat3 import Mat3
from Math.Box2 import Box2
from Node import Node

class SpatialNode2(Node):
    def __init__(self, **args):
        Node.__init__(self, **args)
        
        def transform():
            return Mat3.translation(self.getVar("pos",Vec2(0,0))) * Mat3.rotation(self.getVar("rot",0))
            
        self.setVar("transform",transform)
        
        def globalTransform():
            parent = self.getParentNode()
            if parent:
                return parent.getVar("globalTransform") * self.getVar("transform")
            else:
                return self.getVar("transform")
                
        self.setVar("globalTransform",globalTransform)
        
        def globalBox():
            box = self.getVar("box",Box2())
            for child in self.getChildNodes():
                box.expand(child.getVar("globalBox").transform(child.getVar("transform")))
            return box
            
        self.setVar("globalBox",globalBox)
        
    def getParentNode(self):
        return self.searchAncestors(lambda node: isinstance(node,SpatialNode2))
        
    def getChildNodes(self):
        return self.filterDescendents(lambda node: isinstance(node,SpatialNode2))
        
    def getRelTransform(self, node):
        return self.getVar("globalTransform").inverse() * node.getVar("globalTransform")
        
    def overlaps(self, node):
        return self.getVar("globalBox").overlaps(node.getVar("globalBox").transform(self.getRelTransform(node)))

if __name__ == "__main__":
    import math
    
    n1 = SpatialNode2(vars = dict(box = Box2(Vec2(-1,-1),Vec2(1,1))))
    n2 = SpatialNode2(vars = dict(box = Box2(Vec2(-1,-1),Vec2(1,1))))
    
    assert n1.overlaps(n2)
    
    n2.setVar("pos",Vec2(10,10))
    assert not n1.overlaps(n2)
    
    n1.setVar("pos",Vec2(10,10))
    assert n1.overlaps(n2)
    
    n1.setVar("pos",Vec2(0,0))
    assert not n1.overlaps(n2)
    
    n1.setVar("box",Box2(Vec2(0,0),Vec2(10,10)))
    assert n1.overlaps(n2)

    n1.setVar("rot",math.pi/2)
    assert not n1.overlaps(n2)
