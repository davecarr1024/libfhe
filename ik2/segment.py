from OpenGL.GL import *
import prims
import math
from vec2 import Vec2
from mat3 import Mat3

class Segment:
    def __init__(self, length, childJoint = None):
        self.length = length
        self.parentJoint = None
        self.setChildJoint(childJoint)

    def setChildJoint(self, childJoint):
        self.childJoint = childJoint
        if self.childJoint:
            self.childJoint.parentSegment = self

    def render(self):
        glColor(0,0,1,0.5)
        prims.rect(self.length,10)
        if self.childJoint:
            glPushMatrix()
            glTranslatef(self.length,0,0)
            self.childJoint.render()
            glPopMatrix()

    def getLocalTransform(self):
        return Mat3.translation(Vec2(self.length))

    def getGlobalTransform(self):
        if self.parentJoint:
            return self.parentJoint.getGlobalTransform() * self.getLocalTransform()
        else:
            return self.getLocalTransform()

    def getTipSegment(self):
        if self.childJoint:
            tip = self.childJoint.getTipSegment()
            if tip:
                return tip
        return self

    def getTipTransform(self):
        if self.childJoint:
            return self.getLocalTransform() * self.childJoint.getTipTransform()
        else:
            return self.getLocalTransform()
