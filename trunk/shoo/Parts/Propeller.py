from Shoo import *
from Part import Part

class Propeller(Part):
    def __init__(self,
                 vehicle,
                 density = 1,
                 size = (1,1),
                 pos = (0,0),
                 angle = 0,
                 color = (1,0,0),
                 f = 10000):

        Part.__init__(self,vehicle)
        self.pos = pos
        self.size = size
        self.angle = angle
        self.color = color
        self.f = f
        self.control = 0
        self.propAngle = 0

        shapeDef = b2PolygonDef()
        shapeDef.SetAsBox(size[0]/2.,size[1]/2.,pos,angle)
        shapeDef.density = density
        self.shape = self.vehicle.body.CreateShape(shapeDef)
        self.vehicle.body.SetMassFromShapes()

    def render(self):
        glPushMatrix()
        glTranslate(self.pos[0],self.pos[1],0)
        glRotate(math.degrees(self.angle),0,0,1)
        glScale(self.size[0],self.size[1],1)
        glColor(*self.color)

        Prims.cube()

        glTranslate(0,0.5,0)
        glRotate(math.degrees(self.propAngle),0,1,0)
        glScale(2,0.1,0.1)
        glColor(0,1,0)
        Prims.cube()

        glPopMatrix()
        
    def update(self, time, dtime):
        f = self.control * self.f
        self.propAngle += self.control * dtime * 50
        x = f * math.sin(self.angle) * dtime
        y = f * math.cos(self.angle) * dtime
        force = self.vehicle.body.GetWorldVector((x,y))
        pos = self.vehicle.body.GetWorldPoint(self.pos)
        self.vehicle.body.ApplyForce(force,pos)

    def input(self, name, v, b):
        if name == 'joy0_axis0':
            self.control = v
