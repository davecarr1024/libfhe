from OpenGL.GL import *
import prims
import math
from mat3 import Mat3
from vec2 import Vec2

class Joint:
    def __init__(self, angle, childSegment = None):
        self.angle = angle
        self.parentSegment = None
        self.parentRobot = None
        self.setChildSegment(childSegment)

    def setChildSegment(self, childSegment):
        self.childSegment = childSegment
        if self.childSegment:
            self.childSegment.parentJoint = self

    def render(self):
        glColor(0,1,0,0.5)
        prims.circle(10,8)

        if self.childSegment:
            glPushMatrix()
            glRotatef(math.degrees(self.angle),0,0,-1)
            self.childSegment.render()
            glPopMatrix()

    def getLocalTransform(self):
        return Mat3.rotation(self.angle)

    def getGlobalTransform(self):
        if self.parentSegment:
            return self.parentSegment.getGlobalTransform() * self.getLocalTransform()
        elif self.parentRobot:
            return self.parentRobot.getGlobalTransform() * self.getLocalTransform()
        else:
            return self.getLocalTransform()

    def getTipSegment(self):
        if self.childSegment:
            return self.childSegment.getTipSegment()
        else:
            return None

    def getTipTransform(self, gti = None):
        tipSegment = self.getTipSegment()
        if tipSegment:
            return (gti or self.getGlobalTransform().inverse()) * tipSegment.getGlobalTransform()
        else:
            return Mat3.identity()

    def moveTo(self, pos, time, dtime):
        if self.childSegment and self.childSegment.childJoint:
            self.childSegment.childJoint.moveTo(pos,time,dtime)

        gti = self.getGlobalTransform().inverse()
        tipPos = self.getTipTransform(gti).getTranslation()
        tipPos = Vec2(tipPos.y,-tipPos.x)
        targetPos = (gti * Mat3.translation(pos)).getTranslation()
        angleGain = tipPos.norm().dot(targetPos.norm())
        distGain = (targetPos - tipPos).length() / 50.0
        self.angle += angleGain * distGain * dtime
