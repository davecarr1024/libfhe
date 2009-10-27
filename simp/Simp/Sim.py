# -*- coding: utf-8 -*-

from Entity import Entity
from System import System
import Util
import time

class Sim:
    def __init__(self, **args):
        self.entities = {}
        self.systems = {}
        
        for name, vals in args.get('systems',{}).iteritems():
            self.buildSystem(name,**vals)
            
        for name, vals in args.get('entities',{}).iteritems():
            self.buildEntity(name,**vals)
            
    @staticmethod
    def load(filename, **env):
        return Sim(**Util.deepLoad(filename,**env))
        
    def buildSystem(self, name, **args):
        args['name'] = name
        args['sim'] = self
        return Util.dynLoad(name)(**args)
        
    def buildEntity(self, name, **args):
        args['name'] = name
        args['sim'] = self
        return Entity(**args)
            
    def addEntity(self, entity):
        if not self.hasEntity(entity.name):
            self.entities[entity.name] = entity
            entity.attachToSim(self)
            
    def removeEntity(self, entity):
        if self.hasEntity(entity.name):
            self.entities.pop(entity.name).detachFromSim()
            
    def hasEntity(self, name):
        return name in self.entities
        
    def getEntity(self, name):
        return self.entities.get(name)
        
    def publish(self, cmd, *args, **kwargs):
        for entity in self.entities.itervalues():
            entity.call('msg_%s' % cmd,*args,**kwargs)
            
        for system in self.systems.itervalues():
            system.call('msg_%s' % cmd,*args,**kwargs)
            
    def getSystem(self, name):
        return self.systems.get(name)
        
    def hasSystem(self, name):
        return name in self.systems
        
    def addSystem(self, system):
        if not self.hasSystem(system.name):
            self.systems[system.name] = system
            system.attachToSim(self)
            
    def removeSystem(self, system):
        if self.hasSystem(system.name):
            self.systems.pop(system.name).detachFromSim()
            
    def run(self, maxTime = -1):
        try:
            self.shutdown = False
            startTime = lastTime = currentTime = time.time()
            while not self.shutdown and (maxTime < 0 or currentTime - startTime < maxTime):
                currentTime = time.time()
                dtime = currentTime - lastTime
                lastTime = currentTime
                self.publish('update',currentTime-startTime,dtime)
        finally:
            for entity in self.entities.itervalues():
                entity.call('on_detach')
            for system in self.systems.itervalues():
                system.call('on_detach')

    def serialize(self):
        return dict(entities = dict([(name,entity.serialize()) for name, entity in self.entities.iteritems()]),
                    systems = dict([(name,system.serialize()) for name, system in self.systems.iteritems()]))
                    
    def save(self, filename):
        Util.save(self.serialize(),filename)
        
    def searchEntities(self, c):
        for entity in self.entities.itervalues():
            if c(entity):
                return entity
                
    def filterEntities(self, c):
        return filter(c,self.entities.values())
