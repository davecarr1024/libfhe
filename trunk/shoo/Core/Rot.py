from OpenGL.GL import *
import math

class Rot:
    def __init__(self, a = 0):
        self.a = a

    def __repr__(self):
        return "Rot(%.2f)" % self.a

    def glRotate(self):
        glRotate(math.degrees(self.a),0,0,1)
