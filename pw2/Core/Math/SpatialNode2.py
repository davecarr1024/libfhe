# -*- coding: utf-8 -*-

from Core.Aspect import Aspect
from Core.Math.Vec2 import Vec2
from Core.Math.Rot2 import Rot2
from Core.Math.Mat3 import Mat3

class SpatialNode2(Aspect):
    def get_localTransform(self):
        if not self.getVar("localTransformValid",False):
            self.localTransform = Mat3.translation(self.getVar("pos",Vec2())) *\
                                  Mat3.rotation(self.getVar("rot",Rot2())) *\
                                  Mat3.scale(self.getVar("scale",Vec2(1,1)))
            self.setVar("localTransformValid",True)
        return self.localTransform
               
    def get_globalTransform(self):
        if not all([ent.getVar("globalTransformValid",False) for ent in self.entity.enumerateAncestors(True)]):
            parent = self.entity.searchAncestors(lambda ent: ent.hasVar("globalTransform") and isinstance(ent.getVar("globalTransform",Mat3)))
            if parent:
                self.globalTransform = parent.getVar("globalTransform") * self.getVar("localTransform")
            else:
                self.globalTransform = self.getVar("localTransform")
            self.setVar("globalTransformValid",True)
        return self.globalTransform
        
    def get_invGlobalTransform(self):
        if not self.getVar("invGlobalTransformValid",False):
            self.invGlobalTransform = self.getVar("globalTransform").inverse()
        return self.invGlobalTransform
        
    def set_pos(self, pos):
        self.setVar("globalTransformValid",False)
        self.setVar("invGlobalTransformValid",False)
        self.setVar("localTransformValid",False)

    def set_rot(self, rot):
        self.setVar("globalTransformValid",False)
        self.setVar("invGlobalTransformValid",False)
        self.setVar("localTransformValid",False)

    def set_scale(self, scale):
        self.setVar("globalTransformValid",False)
        self.setVar("invGlobalTransformValid",False)
        self.setVar("localTransformValid",False)
        
    def on_attach(self):
        self.setVar("globalTransformValid",False)
        self.setVar("invGlobalTransformValid",False)
    
    def on_detach(self):
        self.setVar("globalTransformValid",False)
        self.setVar("invGlobalTransformValid",False)
        
    def getRelTransform(self, ent):
        return self.getVar("invGlobalTransform") * ent.getVar("globalTransform",Mat3())
