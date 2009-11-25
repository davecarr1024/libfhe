from Shoo import *

class Vehicle:
    def __init__(self, pos):
        bodyDef = b2BodyDef()
        bodyDef.position = pos
        self.body = shoo.world.CreateBody(bodyDef)

        self.parts = []

        self.addPart("Block", size = (4,2))
        self.addPart("Propeller",pos = (0,1))

    def render(self):
        glPushMatrix()
        glTranslate(self.body.position.x,self.body.position.y,0)
        glRotate(math.degrees(self.body.angle),0,0,1)
        for part in self.parts:
            part.render()
        glPopMatrix()

    def addPart(self, name, **kwargs):
        exec "from Parts.%s import %s as partClass" % (name,name) in locals()
        part = partClass(self,**kwargs)
        self.parts.append(part)
        return part

    def update(self, time, dtime):
        for part in self.parts:
            part.update(time,dtime)

    def input(self, name, v, b):
        for part in self.parts:
            part.input(name,v,b)
