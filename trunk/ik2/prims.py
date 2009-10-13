from OpenGL.GL import *
import math

lists = {}

def serialize(f):
    def wrap(*args):
        global lists
        args = tuple(args)
        if args in lists:
            glCallList(lists[args])
        else:
            l = glGenLists(1)
            glNewList(l,GL_COMPILE_AND_EXECUTE)
            f(*args)
            glEndList()
            lists[args] = l
    return wrap

@serialize
def circle(r, slices):
    glBegin(GL_TRIANGLE_FAN)
    glVertex(0,0)
    for i in range(slices+1):
        th = float(i)/float(slices)*math.pi*2
        glVertex(r*math.cos(th),r*math.sin(th))
    glEnd()

@serialize
def rect(w, h):
    glBegin(GL_QUADS)
    glVertex(0,-h/2)
    glVertex(w,-h/2)
    glVertex(w,h/2)
    glVertex(0,h/2)
    glEnd()
