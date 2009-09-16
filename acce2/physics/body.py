from graphics.prims3d.sceneNode import SceneNode
from world import World
from core.vec3 import Vec3
from core.vec2 import Vec2
from core.quat import Quat
from core.bb import BB

import sys
import math

import pymunk
from OpenGL.GL import *

class Body(SceneNode):
    def getWorld(self):
        return self.searchAncestors(lambda obj: isinstance(obj,World))
    
    def onAttach(self):
        self.defaultVar("static",False)
        self.defaultVar("angle",0)
        self.defaultVar("staticBody",False)
        self.defaultVar("elasticity",0.5)
        self.defaultVar("friction",0.5)
        
        SceneNode.onAttach(self)
        
        self.world = self.getWorld()
        assert self.world, "add a world above bodies"
        
        self.body, self.shape = self.makeBody()
        
        if self.body:
            self.body.position = pymunk.Vec2d(self.position.x,self.position.y)
            self.body.angle = self.angle
            
        if self.shape:
            self.shape.elasticity = self.elasticity
            self.shape.friction = self.friction
            self.shape.collision_type = 0
            self.shape.object = self
        
        if self.body and self.shape and not self.staticBody:
            self.world.space.add(self.body,self.shape)
        elif self.shape and self.staticBody:
            self.world.space.add(self.shape)
            
    def makeBody(self, shapeOffset):
        pass
        
    def onDetach(self):
        if self.world:
            if self.body:
                self.world.space.remove(self.body)
            if self.shape:
                self.world.space.remove(self.shape)
        self.world = None
        self.body = None
        self.shape = None
        
    def transform(self):
        self.angle = self.body.angle
        self.position = Vec3(self.body.position.x,self.body.position.y)
        self.rotation = Quat.fromAxisAngle(Vec3.UNIT_Z,self.angle)
        SceneNode.transform(self)

    def applyForce(self, f, r = None):
        if self.body:
            self.body.apply_force(f.toTuple(),(r or Vec2()).toTuple())

    def resetForces(self):
        if self.body:
            self.body.reset_forces()

    def applyImpulse(self, j, r = None):
        if self.body:
            self.body.apply_impulse(j.toTuple(),(r or Vec2()).toTuple())

    def setSurfaceVelocity(self, v):
        if self.shape:
            self.shape.surface_velocity = pymunk.Vec2d(v.x,v.y)

    def setPosition(self, v):
        if self.body:
            self.body.position = pymunk.Vec2d(v.x,v.y)
            self.position = Vec3(v.x,v.y,self.position.z)

    def setVelocity(self, v):
        if self.body:
            self.body.velocity = pymunk.Vec2d(v.x,v.y)

    def localToWorld(self, v):
        if self.body:
            wv = self.body.local_to_world(pymunk.Vec2d(v.x,v.y))
            return Vec2(wv.x,wv.y)

    def worldToLocal(self, v):
        if self.body:
            lv = self.body.world_to_local(pymunk.Vec2d(v.x,v.y))
            return Vec2(lv.x,lv.y)

    def getBB(self):
        return BB()

    def getWorldBB(self):
        corners = map(self.localToWorld,self.getBB().getCorners())
        bb = BB(corners[0])
        map(bb.expand,corners[1:])
        return bb
