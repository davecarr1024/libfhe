from Core.Vec2 import Vec2
from Core.Rot import Rot

from OpenGL.GL import *

class SceneNode2:
    def __init__(self):
        self.pos = Vec2(0,0)
        self.scale = Vec2(1,1)
        self.rot = Rot()
        self.children = []
        self.parent = None
        self.material = None

    def attachToParent(self, parent):
        if parent != self.parent:
            self.detachFromParent()
            self.parent = parent
            if self.parent:
                self.parent.addChild(self)

    def detachFromParent(self):
        if self.parent:
            parent = self.parent
            self.parent = None
            parent.removeChild(self)

    def addChild(self, node):
        if node not in self.children:
            self.children.append(node)
            node.attachToParent(self)

    def removeChild(self, node):
        if node in self.children:
            self.children.remove(node)
            node.detachFromParent()

    def geom(self):
        pass

    def transform(self):
        glPushMatrix()
        self.pos.glTranslate()
        self.rot.glRotate()
        self.scale.glScale()

    def untransform(self):
        glPopMatrix()

    def render(self):
        self.transform()
        
        if self.material:
            self.material.bind()

        self.geom()

        if self.material:
            self.material.unbind()

        for child in self.children:
            child.render()

        self.untransform()
