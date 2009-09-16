from core.object import Object
from physics.circle import Circle
from physics.rect import Rect
from physics.poly import Poly as PhysicsPoly
from graphics.prims3d.cylinder import Cylinder
from graphics.prims3d.cube import Cube
from graphics.prims3d.poly import Poly as GraphicsPoly
from core.vec3 import Vec3
from core.vec2 import Vec2

import random
import math

class BallDropper(Object):
    def onAttach(self):
        self.defaultVar("ballTime",1)
        self.defaultVar("maxBalls",50)
        self.defaultVar("size",2)
        self.defaultVar("miny",-10)
        self.lastBallTime = None
        self.balls = []

    def msg_tick(self,args):
        time = args['time']
        if not self.lastBallTime:
            self.lastBallTime = time

        if time - self.lastBallTime > self.ballTime and len(self.balls) < self.maxBalls:
            self.lastBallTime += self.ballTime

            t = random.randint(0,2)
            pos = Vec3(random.uniform(-10,10),30,0)
            color = (random.random(),random.random(),random.random())
            material = dict(color = color,
                            texture = "test.jpg")

            if t == 0:
                if random.randint(0,10) == 0:
                    radius = self.size * 2
                else:
                    radius = random.uniform(self.size/4,self.size/2)

                ball = Circle(parent = self,
                              vars = dict(position = pos,
                                          radius = radius,
                                          mass = math.pi * radius * radius))

                Cylinder(parent = ball,
                         vars = dict(material = material,
                                     scale = Vec3(radius,radius,1)))

            elif t == 1:
                size = random.uniform(self.size/2,self.size)

                verts = [Vec2(-size/2,0),Vec2(0,size),Vec2(size/2,0)]
                
                ball = PhysicsPoly(parent = self,
                                  vars = dict(vertices = verts,
                                              position = pos,
                                              angle = random.uniform(0,math.pi*2)))
                GraphicsPoly(parent = ball,
                             vars = dict(material = material,
                                         vertices = verts))

            else:
                w = random.uniform(self.size/2,self.size)
                h = random.uniform(self.size/2,self.size)

                ball = Rect(parent = self,
                            vars = dict(position = pos,
                                        bodySize = Vec2(w,h),
                                        mass = w * h,
                                        angle = random.uniform(0,math.pi*2)))
                Cube(parent = ball,
                     vars = dict(scale = Vec3(w,h,1),
                                 material = material))

            self.balls.append(ball)

        for ball in self.balls:
            if ball.position.y < self.miny:
                ball.delete()
                self.balls.remove(ball)
