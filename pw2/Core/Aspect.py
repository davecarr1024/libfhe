# -*- coding: utf-8 -*-

class Aspect:
    def init(self, name, entity):
        self.name = name
        self.entity = entity
        self.funcs = dict([(name,getattr(self,name)) for name in dir(self) if callable(getattr(self,name))])
        self.updatePath()
        
    def hasFunc(self, cmd):
        return cmd in self.funcs
        
    def call(self, cmd, *args, **kwargs):
        assert self.hasFunc(cmd)
        return self.funcs[cmd](*args,**kwargs)
        
    def tryCall(self, cmd, default, *args, **kwargs):
        if self.hasFunc(cmd):
            return self.funcs[cmd](*args,**kwargs)
        else:
            return default
            
    def attachToEntity(self, entity):
        if entity != self.entity:
            self.detachFromEntity()
            self.entity = entity
            if self.entity:
                self.entity.addAspect(self)
            self.updatePath()
                
    def detachFromEntity(self):
        if self.entity:
            entity = self.entity
            self.entity = None
            entity.removeAspect(self)
            self.updatePath()
            
    def updatePath(self):
        self.path = "%s.%s" % (self.entity.path,self.name)
            
    def log(self, *v):
        print "%s: %s" % (self.path,' '.join(map(str,v)))
