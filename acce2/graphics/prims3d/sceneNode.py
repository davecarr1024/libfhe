from core.object import Object
from core.vec3 import Vec3
from core.quat import Quat
from graphics.materialManager import materialManager
import math

from OpenGL.GL import *

class SceneNode(Object):
    genNode = None

    def onAttach(self):
        self.defaultVar("position",Vec3())
        self.defaultVar("scale",Vec3(1,1,1))
        self.defaultVar("rotation",Quat())
        self.defaultVar("material",{})
        self.defaultVar("static",True)
        self.list = None

    def transform(self):
        self.position.translate()
        self.rotation.rotate()
        self.scale.scale()

    def msg_render3d(self, args):
        glPushMatrix()
        self.transform()

        if not args.get('picking') and self.static and not SceneNode.genNode:
            if self.list:
                glCallList(self.list)
                return False
            else:
                self.list = glGenLists(1)
                glNewList(self.list,GL_COMPILE_AND_EXECUTE)
                SceneNode.genNode = self

        materialManager.bind(self.material)

        if args.get('picking'):
            args['pickObjects'].append(self)
            glPushName(len(args['pickObjects']))
        
        self.geom()

        if args.get('picking'):
            glPopName()

    def geom(self):
        pass

    def unmsg_render3d(self, args):
        if args.get('picking') or not self.static or SceneNode.genNode:
            glPopMatrix()
            materialManager.unbind()
        if SceneNode.genNode == self:
            glEndList()
            SceneNode.genNode = None
