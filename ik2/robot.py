from OpenGL.GL import *
from mat3 import Mat3
from vec2 import Vec2
import prims

class Robot:
    def __init__(self, target, pos, rootJoint = None):
        self.target = target
        self.pos = pos
        self.setRootJoint(rootJoint)

    def setRootJoint(self, rootJoint):
        self.rootJoint = rootJoint
        if self.rootJoint:
            self.rootJoint.parentRobot = self

    def render(self):
        if self.rootJoint:
            glPushMatrix()
            self.pos.translate()
            self.rootJoint.render()
            glPopMatrix()

        if self.target:
            self.target.render()

    def getLocalTransform(self):
        return Mat3.translation(self.pos)

    def getGlobalTransform(self):
        return self.getLocalTransform()

    def getTipSegment(self):
        if self.rootJoint:
            return self.rootJoint.getTipSegment()
        else:
            return None

    def getTipPosition(self):
        tip = self.getTipSegment()
        if tip:
            return tip.getGlobalTransform().getTranslation()
        else:
            return Vec2()

    def tick(self, time, dtime):
        if self.target and self.rootJoint:
            self.rootJoint.moveTo(self.target.pos, time, dtime)
