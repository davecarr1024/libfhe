# -*- coding: utf-8 -*-
from Core.Node import Node
from Body import Body
from World import World

import pymunk

class Joint(Node):
    def getWorld(self):
        return self.searchAncestors(lambda obj: isinstance(obj,World))

    def onAttach(self):
        self.world = self.getWorld()
        assert self.world, "add a world above joints"
        
        body1 = self.getVar("body1",None)
        body2 = self.getVar("body2",None)

        assert body1 and body2 and \
               isinstance(body1,Body) and isinstance(body2,Body), \
               "add body1 and body2 to joint"

        assert self.body1.body and self.body2.body

        self.joint = self.makeJoint(self.body1.body,self.body2.body)
        if self.joint:
            self.world.space.add(self.joint)

    def onDetach(self):
        if self.joint and self.world:
            self.world.space.remove(self.joint)
        self.joint = None
        self.world = None

    def makeJoint(self, a, b):
        pass
