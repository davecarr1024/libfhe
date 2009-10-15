# -*- coding: utf-8 -*-
from Core.Node import Node
from Core.Math.Vec2 import Vec2

import pymunk

class World(Node):
    def onAttach(self):
        pymunk.init_pymunk()
        self.space = pymunk.Space()
        self.space.gravity = pymunk.Vec2d(*self.getVar("gravity",Vec2(0,-100)))
        
        self.lastTick = None
        self.fps = self.getVar('fps',100)
        if self.fps:
            self.tickTime = 1.0 / self.fps
        else:
            self.tickTime = 0

        self.space.add_collisionpair_func(0,0,self.collision)

    def collision(self, body1, body2, contacts, normal, data):
        if hasattr(body1,"node") and hasattr(body2,"node"):
            node1 = body1.node
            node2 = body2.node
            self.publish("collision",node1,node2,contacts)
        return True
        
    def msg_update(self, time, dtime):
        if self.tickTime:
            if not self.lastTick:
                self.lastTick = time
            if time - self.lastTick > self.tickTime:
                self.lastTick += self.tickTime
                self.space.step(self.tickTime)
        else:
            self.space.step(dtime)
