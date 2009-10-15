# -*- coding: utf-8 -*-
from Graphics.Prims3.SceneNode import SceneNode
from World import World
from Core.Math.Vec3 import Vec3
from Core.Math.Vec2 import Vec2d
from Core.Math.Quat import Quat

import math

import pymunk
from OpenGL.GL import *

class Body(SceneNode):
    def getWorld(self):
        return self.searchAncestors(lambda obj: isinstance(obj,World))
    
    def onAttach(self):
        SceneNode.onAttach(self)
        
        self.world = self.getWorld()
        assert self.world, "add a world above bodies"
        
        self.body, self.shape = self.makeBody()
        
        if self.body:
            self.body.position = pymunk.Vec2d(*self.getVar("pos",Vec2()))
            self.body.angle = self.getVar("rot",0)
            
        if self.shape:
            self.shape.elasticity = self.getVar("elasticity",0.5)
            self.shape.friction = self.getVar("friction",0.5)
            self.shape.collision_type = 0
            self.shape.node = self
        
        staticBody = self.getVar("staticBody",False)
        if self.body and self.shape and not staticBody:
            self.world.space.add(self.body,self.shape)
        elif self.shape and staticBody:
            self.world.space.add(self.shape)
            
    def makeBody(self):
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
        if self.body:
            self.setVar("pos",Vec3(self.body.position.x,self.body.position.y,0))
            self.setVar("rot",Quat.fromAxisAngle(Vec3.UNIT_Z,self.body.angle))
        SceneNode.transform(self)

    def applyForce(self, f, r = None):
        if self.body:
            self.body.apply_force(f,r or Vec2())

    def resetForces(self):
        if self.body:
            self.body.reset_forces()

    def applyImpulse(self, j, r = None):
        if self.body:
            self.body.apply_impulse(j,r or Vec2())

    def set_surfaceVelocity(self, v):
        if self.shape:
            self.shape.surface_velocity = pymunk.Vec2d(*v)

    def set_pos(self, v):
        if self.body:
            self.body.position = pymunk.Vec2d(*v)

    def set_vel(self, v):
        if self.body:
            self.body.velocity = pymunk.Vec2d(*v)

    def localToWorld(self, v):
        if self.body:
            wv = self.body.local_to_world(pymunk.Vec2d(v.x,v.y))
            return Vec2(wv.x,wv.y)

    def worldToLocal(self, v):
        if self.body:
            lv = self.body.world_to_local(pymunk.Vec2d(v.x,v.y))
            return Vec2(lv.x,lv.y)
