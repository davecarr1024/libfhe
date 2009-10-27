# -*- coding: utf-8 -*-

from Aspect import Aspect
import Util

class Entity:
    nameCounts = {}
    
    @staticmethod
    def makeName(name):
        if name not in Entity.nameCounts:
            Entity.nameCounts[name] = 1
            return name
        else:
            Entity.nameCounts[name] += 1
            return "%s_%d" % (name,Entity.nameCounts[name])
    
    def __init__(self, **args):
        self.name = Entity.makeName(args.get('name','Ent'))
        self.sim = None
        self.vars = {}
        self.aspects = {}
        
        self.attachToSim(args.get('sim',None))
        
        for name, val in args.get('vars',{}).iteritems():
            self.setVar(name,val)
            
        for name, vals in args.get('aspects',{}).iteritems():
            self.buildAspect(name,**vals)
            
    def buildAspect(self, name, **args):
        args['name'] = name
        args['entity'] = self
        return Util.dynLoad(name)(**args)
            
    def setVar(self, name, val):
        self.vars[name] = val
        self.call('set_%s' % name, val)
        
    def getVar(self, name, default = None):
        val = self.call('get_%s' % name) or self.vars.get(name,default)
        if callable(val):
            return val()
        elif isinstance(val,str) and val.startswith('='):
            return Util.deepEval(val[1:],self = self)
        else:
            return val
            
    def defaultVar(self, name, val):
        self.setVar(self.getVar(name,val))
        
    def hasVar(self, name):
        return name in self.vars or self.hasFunc('get_%s' % name)

    def getFuncNames(self):
        return sum([aspect.funcs.keys() for aspect in self.aspects.itervalues()],[])

    def getVarNames(self):
        return self.vars.keys() + filter(lambda name: name.startswith('get_'),self.getFuncNames())
        
    def hasFunc(self, name):
        return any([aspect.hasFunc(name) for aspect in self.aspects.itervalues()])
        
    def call(self, name, *args, **kwargs):
        for aspect in self.aspects.itervalues():
            if aspect.hasFunc(name):
                return aspect.call(name,*args,**kwargs)
         
    def hasAspect(self, name):
        return name in self.aspects
        
    def getAspect(self, name):
        return self.aspects.get(name)
         
    def addAspect(self, aspect):
        if not self.hasAspect(aspect.name):
            self.aspects[aspect.name] = aspect
            aspect.attachToEntity(self)
            
    def removeAspect(self, aspect):
        if self.hasAspect(aspect.name):
            self.aspects.pop(aspect.name).detachFromEntity()
            
    def attachToSim(self, sim):
        if sim != self.sim:
            self.detachFromSim()
            self.sim = sim
            if self.sim:
                self.sim.addEntity(self)
                
    def detachFromSim(self):
        if self.sim:
            sim = self.sim
            self.sim = None
            sim.removeEntity(self)
            
    def serialize(self):
        data = {}
        return dict(vars = dict([(name,self.getVar(name)) for name in self.getVarNames()]),
                    aspects = dict([(name,aspect.serialize()) for name, aspect in self.aspects.iteritems()]))
