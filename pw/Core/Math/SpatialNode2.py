# -*- coding: utf-8 -*-

from Vec2 import Vec2
from Mat3 import Mat3
from Box2 import Box2
from Core.Node import Node
import math

class SpatialNode2(Node):
    def __init__(self, **args):
        Node.__init__(self, **args)
        
        self.inverseGlobalTransformValid = False
        self.globalTransformValid = False 
        self.localTransformValid = False
        self.globalBoxValid = False
        
        def transform():
            if not self.localTransformValid:
                self.localTransformValid = True
                self.localTransform = Mat3.translation(self.getVar("pos",Vec2(0,0))) * \
                                      Mat3.rotation(self.getVar("rot",0)) * \
                                      Mat3.scale(self.getVar("scale",Vec2(1,1)))
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
        
        def inverseGlobalTransform():
            if not self.inverseGlobalTransformValid:
                self.inverseGlobalTransformValid = True
                self.inverseGlobalTransform = self.getVar("globalTransform").inverse()
            return self.inverseGlobalTransform
        
        self.setVar("inverseGlobalTransform",inverseGlobalTransform)
        
        def globalBox():
            if not self.globalBoxValid:
                self.globalBoxValid = True
                self.globalBox = self.getVar("box",Box2())
                for child in self.getChildNodes():
                    self.globalBox.expand(child.getVar("globalBox").transform(child.getVar("transform")))
            return self.globalBox
            
        self.setVar("globalBox",globalBox)
        
    def set_pos(self, pos):
        self.inverseGlobalTransformValid = self.globalTransformValid = self.localTransformValid = False
        
    def set_rot(self, rot):
        self.inverseGlobalTransformValid = self.globalTransformValid = self.localTransformValid = False
        
    def set_box(self, box):
        for node in self.getAncestors():
            node.globalBoxValid = False
        
    def onAttach(self):
        self.inverseGlobalTransformValid = self.globalTransformValid = False
        
    def onDetach(self):
        self.inverseGlobalTransformValid = self.globalTransformValid = False
        
    def getParentNode(self):
        return self.searchAncestors(lambda node: isinstance(node,SpatialNode2))
        
    def getAncestors(self):
        return self.filterAncestors(lambda node: isinstance(node,SpatialNode2), True)
        
    def getChildNodes(self):
        return self.filterDescendents(lambda node: isinstance(node,SpatialNode2))
        
    def allAncestorsGlobalTransformValid(self):
        return all([node.globalTransformValid for node in self.getAncestors()])
        
    def getRelTransform(self, node):
        return self.getVar("globalTransform").inverse() * node.getVar("globalTransform")
        
    def overlaps(self, node):
        return self.getVar("globalBox").overlaps(node.getVar("globalBox").transform(self.getRelTransform(node)))
        
    def collideVec(self, v):
        return True
        
    def collide(self, x):
        colls = sum([child.collide(x) for child in self.children.itervalues()],[])
        
        if colls:
            return colls
        elif isinstance(x,Vec2):
            sx = self.getVar("inverseGlobalTransform") * x
            if self.getVar("globalBox").containsVec(sx) and self.collideVec(sx):
                return [self]
            else:
                return []
        else:
            raise TypeError

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
