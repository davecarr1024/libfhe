from core.object import Object
from core.vec2 import Vec2
from core.mat3 import Mat3
from graphics.materialManager import materialManager
import math

from OpenGL.GL import *

class SceneNode(Object):
    genNode = None
    
    def onAttach(self):
        self.defaultVar("position",Vec2())
        self.defaultVar("scale",Vec2(1,1))
        self.defaultVar("rotation",0)
        self.defaultVar("material",{})
        self.defaultVar("static",True)
        self.list = None
    
    def msg_render2d(self, args):
        glPushMatrix()
        
        self.transform()
        
        if not args.get('picking') and self.static and not SceneNode.genNode:
            if self.list:
                glCallList(self.list)
                return False
            else:
                self.list = glGenLists(1)
                glNewList(self.list,GL_COMPILE_AND_EXECUTE)
                SceneNode.genNode = self
            
        materialManager.bind(self.material)
        
        if args.get('picking'):
            args['pickObjects'].append(self)
            glPushName(len(args['pickObjects']))
        
        self.geom()

        if args.get('picking'):
            glPopName()
        
    def transform(self):
        self.position.translate()
        glRotatef(math.degrees(self.rotation),0,0,1)
        self.scale.scale()
        
    def geom(self):
        pass

    def unmsg_render2d(self, args):
        if args.get('picking') or not self.static or SceneNode.genNode:
            glPopMatrix()
            materialManager.unbind()
        if SceneNode.genNode == self:
            glEndList()
            SceneNode.genNode = None

    def getParentSceneNode(self):
        return self.searchAncestors(lambda obj: isinstance(obj,SceneNode))

    def getLocalTransform(self):
        s = Mat3.scale(self.scale)
        r = Mat3.rotation(self.rotation)
        t = Mat3.translation(self.position)
        return t * r * s
    
    def getParentTransform(self):
        parent = self.getParentSceneNode()
        if parent:
            return parent.getGlobalTransform()
        else:
            return Mat3.identity()

    def getGlobalTransform(self):
        parent = self.getParentSceneNode()
        if parent:
            return self.getLocalTransform() * parent.getGlobalTransform()
        else:
            return self.getLocalTransform()
        
    def globalToParent(self, v):
        for obj in list(reversed(self.filterAncestors(lambda obj: isinstance(obj,SceneNode)))):
            v = (v - obj.position) / obj.scale
        return v
        
    def parentToLocal(self, v):
        return (v - self.position) / self.scale
    
    def globalToLocal(self, v):
        return self.parentToLocal(self.globalToParent(v))
    
    def parentToGlobal(self, v):
        return self.getParentTransform() * v
    
    def localToGlobal(self, v):
        return self.getGlobalTransform() * v
    
    def localToParent(self, v):
        return self.getLocalTransform() * v
