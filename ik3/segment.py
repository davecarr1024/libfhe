from OpenGL.GL import *
import prims
from vec3 import Vec3
from mat4 import Mat4

class Segment:
    def __init__(self, length, childJoint):
        self.length = length
        self.parentJoint = None
        self.setChildJoint(childJoint)

    def setChildJoint(self, childJoint):
        self.childJoint = childJoint
        if self.childJoint:
            self.childJoint.parentSegment = self

    def render(self):
        glColor(0,0,1)
        prims.cylinder(0.3,self.length,8)
        if self.childJoint:
            glPushMatrix()
            glTranslatef(self.length,0,0)
            self.childJoint.render()
            glPopMatrix()

    def getLocalTransform(self):
        return Mat4.translation(Vec3(self.length,0,0))

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
