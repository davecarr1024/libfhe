from Shoo import *

class Vec3:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
        self.z = z

    def glTranslate(self):
        glTranslate(self.x,self.y,self.z)

    def glScale(self):
        glScale(self.x,self.y,self.z)

