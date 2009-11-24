from Core.MessageServer import messageServer
from Core.OptionServer import optionServer
from Core.Vec2 import Vec2

import pymunk

class World:
    def __init__(self):
        self.gravity = optionServer.get('gravity',Vec2(0,-100))
        pymunk.init_pymunk()
        self.space = pymunk.Space()
        self.space.gravity = pymunk.Vec2d(*self.gravity)

        self.lastTick = None
        self.fps = optionServer.get('physicsFps',100)
        if self.fps:
            self.tickTime = 1.0 / self.fps
        else:
            self.tickTime = 0

        messageServer.subscribe("update",self.update)

    def update(self, time, dtime):
        if self.tickTime:
            if not self.lastTick:
                self.lastTick = time
            if time - self.lastTick > self.tickTime:
                self.lastTick += self.tickTime
                self.space.step(self.tickTime)
        else:
            self.space.step(dtime)
