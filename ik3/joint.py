from OpenGL.GL import *
import prims
import math
from vec3 import Vec3
from mat4 import Mat4
from quat import Quat
from plane import Plane

class Joint:
    def __init__(self, axis, angle, childSegment):
        self.axis = axis
        self.angle = angle
        self.parentRobot = None
        self.parentSegment = None
        self.setChildSegment(childSegment)

    def setChildSegment(self, childSegment):
        self.childSegment = childSegment
        if self.childSegment:
            self.childSegment.parentJoint = self

    def render(self):
        glColor(0,1,0)
        prims.sphere(0.5,8,8)
        if self.childSegment:
            glPushMatrix()
            glRotate(math.degrees(self.angle),self.axis.x,self.axis.y,self.axis.z)
            self.childSegment.render()
            glPopMatrix()

    def getLocalTransform(self):
        return Mat4.rotation(Quat.fromAxisAngle(self.axis,self.angle))

    def getGlobalTransform(self):
        if self.parentRobot:
            return self.parentRobot.getGlobalTransform() * self.getLocalTransform()
        elif self.parentSegment:
            return self.parentSegment.getGlobalTransform() * self.getLocalTransform()
        else:
            return self.getLocalTransform()

    def getTipSegment(self):
        if self.childSegment:
            return self.childSegment.getTipSegment()

    def getTipTransform(self):
        tipSegment = self.getTipSegment()
        if tipSegment:
            return self.getGlobalTransform().inverse() * tipSegment.getGlobalTransform()

    def moveTo(self, pos, time, dtime):
        if self.childSegment and self.childSegment.childJoint:
            self.childSegment.childJoint.moveTo(pos,time,dtime)
        
        gt = self.getGlobalTransform()
        gti = gt.inverse()

        jointAxis = self.axis.norm()
        jointPos = gt.getTranslation()
        jointPlane = Plane.fromPointNormal(jointPos,jointAxis)

        targetPos = (gti * Mat4.translation(pos)).getTranslation()
        projectedTargetPos = jointPlane.projectPoint(targetPos)
        
        tipPos = self.getTipTransform().getTranslation()
        projectedTipPos = jointPlane.projectPoint(tipPos)

##        d = projectedTipPos.norm().dot(projectedTargetPos.norm())
##        if d < 1 - 1e-4:
##            newAngle = math.acos(d)
##        else:
##            newAngle = 0

        r = projectedTipPos.norm().getRotationTo(projectedTargetPos.norm())
        newAxis, newAngle = r.toAxisAngle()
        if newAxis.dot(jointAxis) < 0:
            newAngle *= -1

        self.angle += newAngle * dtime * 10
