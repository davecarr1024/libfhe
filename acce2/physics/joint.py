from core.object import Object
from body import Body
from world import World

import pymunk

class Joint(Object):
    def getWorld(self):
        return self.searchAncestors(lambda obj: isinstance(obj,World))

    def onAttach(self):
        Object.onAttach(self)

        self.defaultVar("body1",None)
        self.defaultVar("body2",None)

        self.world = self.getWorld()
        assert self.world, "add a world above joints"

        assert self.body1 and self.body2 and \
               isinstance(self.body1,Body) and isinstance(self.body2,Body), \
               "add bodies to joint"

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
