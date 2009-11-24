from World import world
from Graphics.SceneNode2 import SceneNode2

import pymunk

class Body(SceneNode2):
    def __init__(self):
        self.body = pymunk.Body(pymunk.inf,pymunk.inf)
        self.shapes = []

    def addShape(self, shape):
        if shape not in self.shapes:
            self.shapes.append(shape)
            shape.attachToBody(self)
            self.calc()

    def removeShape(self, shape):
        if shape in self.shapes:
            self.shapes.remove(shape)
            shape.detachFromBody()
            self.calc()

    def calc(self):
        self.body.mass = sum([shape.mass for shape in self.shapes]) or pymunk.inf
        self.body.moment = sum([shape.moment for shape in self.shapes]) or pymunk.inf
