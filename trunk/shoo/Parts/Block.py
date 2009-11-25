from Shoo import *
from Part import Part

class Block(Part):
    def __init__(self,
                 vehicle,
                 size = (1,1),
                 density = 1,
                 pos = (0,0),
                 angle = 0,
                 color = (0,0,1)):
        
        Part.__init__(self,vehicle)
        self.pos = pos
        self.size = size
        self.angle = angle
        self.color = color

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

        glPopMatrix()
