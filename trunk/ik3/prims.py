from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
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
def cylinder(r, h, segments):
    x = []
    z = []
    for i in range(segments+1):
        th = float(i)/float(segments)*math.pi*2
        x.append(r * math.cos(th))
        z.append(r * math.sin(th))

    glBegin(GL_QUAD_STRIP)
    for i in range(segments+1):
        glVertex3f(0,x[i],z[i])
        glVertex3f(h,x[i],z[i])
    glEnd()

@serialize
def sphere(r, stacks, slices):
    glutSolidSphere(r,stacks,slices)
