from OpenGL.GL import *
from segment import Segment
from joint import Joint
from vec3 import Vec3
from mat4 import Mat4

class Robot:
    def __init__(self, pos, target, rootJoint):
        self.pos = pos
        self.target = target
        self.setRootJoint(rootJoint)

    def setRootJoint(self, rootJoint):
        self.rootJoint = rootJoint
        if self.rootJoint:
            self.rootJoint.parentRobot = None
        
    def render(self):
        if self.rootJoint:
            glPushMatrix()
            self.pos.translate()
            self.rootJoint.render()
            glPopMatrix()

        if self.target:
            self.target.render()

    def tick(self, time, dtime):
        if self.target and self.rootJoint:
            self.rootJoint.moveTo(self.target.pos, time, dtime)

    def getLocalTransform(self):
        return Mat4.translation(self.pos)

    def getGlobalTransform(self):
        return self.getLocalTransform()

    def getTipSegment(self):
        if self.rootJoint:
            return self.rootJoint.getTipSegment()
