from physics.rect import Rect
from physics.pinJoint import PinJoint
from graphics.prims3d.cube import Cube
from core.vec2 import Vec2
from core.vec3 import Vec3

class Player(Rect):
    def onAttach(self):
        Rect.onAttach(self)

        self.defaultVar("playerNum",0)
        self.defaultVar("flyForce",20)
        self.defaultVar("moveForce",40)
        self.defaultVar("jumpForce",30)
        self.defaultVar("kickForce",10)
        self.defaultVar("friction",1)
        self.defaultVar("goFastFactor",0.5)
        self.defaultVar("lives",5)
        self.defaultVar("score",0)
        self.alive = True
        self.dleft = 0
        self.dright = 0
        self.goFast = 1

        self.defaultVar("material",{})
        scale = Vec3(self.bodySize.x,self.bodySize.y,1)
        self.graphics = Cube(parent = self,
                             vars = dict(scale = scale,
                                         material = self.material))

        self.flying = True
        self.grabbedObject = None
        self.canGrab = False
        self.grabJoint = None

    def msg_collision(self, args):
        if self in args['objects']:
            obj = filter(lambda obj: obj != self,args['objects'])[0]
            if not obj.staticBody and not self.grabbedObject and self.canGrab:
                selfBB = self.getWorldBB()
                objBB = obj.getWorldBB()
                
                if objBB.max.y - selfBB.min.y > 0.5:
                    self.grabbedObject = obj
                    self.grabMass = obj.body.mass
                    obj.body.mass = 1e-5

                    jointPos = obj.position
                    a1 = self.worldToLocal(jointPos)
                    a2 = obj.worldToLocal(jointPos)
                    self.grabJoint = PinJoint(parent = self,
                                              vars = dict(body1 = self,
                                                          body2 = obj,
                                                          anchor1 = a1,
                                                          anchor2 = a2))

            elif obj != self.grabbedObject:
                self.flying = False

    def msg_tick(self, args):
        if self.flying:
            dtime = args['dtime']
            dmove = self.dright - self.dleft
            f = dmove * self.goFast * self.flyForce * dtime
            self.applyImpulse(Vec2(f,0))
            self.setSurfaceVelocity(Vec2())
        else:
            f = (self.dleft - self.dright) * self.goFast * self.moveForce
            self.setSurfaceVelocity(Vec2(f,0))

    def msg_goFast(self, args):
        self.goFast = args.get('f',0) * self.goFastFactor + 1
        if args.get('b'):
            self.canGrab = True
        else:
            if self.grabJoint:
                self.grabJoint.delete()
                self.grabJoint = None
            if self.grabbedObject and self.grabbedObject.body:
                self.grabbedObject.body.mass = self.grabMass
                d = (self.grabbedObject.position - self.position).norm()
                self.grabbedObject.applyImpulse(d * self.kickForce)
            self.grabbedObject = None
            self.canGrab = False

    def msg_moveRight(self, args):
        self.dright = args.get('f',0)

    def msg_moveLeft(self, args):
        self.dleft = args.get('f',0)

    def msg_jump(self, args):
        if args.get('b') and not self.flying:
            self.flying = True
            self.applyImpulse(Vec2(0,self.jumpForce * self.goFast))
