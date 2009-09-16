from core.object import Object
from core.vec2 import Vec2
from core.optionServer import optionServer

import pymunk

class World(Object):
    def onAttach(self):
        self.defaultVar("gravity",optionServer.get('gravity',Vec2(0,-100)))
        pymunk.init_pymunk()
        self.space = pymunk.Space()
        self.space.gravity = pymunk.Vec2d(self.gravity.x,self.gravity.y)
        
        self.lastTick = None
        self.fps = optionServer.get('physicsFps',100)
        if self.fps:
            self.tickTime = 1.0 / self.fps
        else:
            self.tickTime = 0

        self.space.add_collisionpair_func(0,0,self.collision)

    def collision(self, body1, body2, contacts, normal, data):
        if hasattr(body1,"object") and hasattr(body2,"object"):
            obj1 = body1.object
            obj2 = body2.object
            self.localPublish("collision",objects = (obj1,obj2), contacts = contacts)
        return True
        
    def msg_tick(self, args):
        if self.tickTime:
            time = args['time']
            if not self.lastTick:
                self.lastTick = time
            if time - self.lastTick > self.tickTime:
                self.lastTick += self.tickTime
                self.space.step(self.tickTime)
        else:
            self.space.step(args['dtime'])
