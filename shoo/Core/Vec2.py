from OpenGL.GL import *

class Vec2:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

    def __repr__(self):
        return "Vec2(%.2f,%.2f)" % (self.x,self.y)

    def __getitem__(self, i):
        if i == 0:
            return self.x
        elif i == 1:
            return self.y
        else:
            raise IndexError

    def __setitem__(self, i, f):
        if i == 0:
            self.x = f
        elif i == 1:
            self.y = f
        else:
            raise IndexError

    def glTranslate(self):
        glTranslate(self.x,self.y,0)

    def glScale(self):
        glScale(self.x,self.y,1)
