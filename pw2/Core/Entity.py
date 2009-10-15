# -*- coding: utf-8 -*-

from Core.Math.Vec2 import Vec2
from Core.Math.Rot2 import Rot2
from Core.Math.Mat3 import Mat3
from Core.Math.Box2 import Box2

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
        self.path = "/"
        self.parent = None
        self.children = {}
        self.vars = {}
        self.aspects = {}
        
        for name, val in args.get('vars',{}).iteritems():
            self.setVar(name,val)
            
        for name in Util.flatten(args.get('aspects',[])):
            self.buildAspect(name)
            
        self.attachToParent(args.get('parent',None))
            
        for name, val in args.get('children',{}).iteritems():
            val.setdefault('name',name)
            val['parent'] = self
            self.addChild(Entity(**val))
        
    def __repr__(self):
        return "<Entity %s>" % self.name
            
    def updatePath(self):
        if self.parent:
            if self.parent.parent:
                self.path = self.parent.path + "/" + self.name
            else:
                self.path = "/" + self.name
        else:
            self.path = "/"
            
    def loadChild(self, filename):
        data = Util.deepLoad(filename,Vec2,Rot2,Mat3,Box2)
        data['parent'] = self
        self.addChild(Entity(**data))
            
    def hasFunc(self, cmd):
        return any([aspect.hasFunc(cmd) for aspect in self.aspects.itervalues()])
        
    def call(self, cmd, *args, **kwargs):
        assert self.hasFunc(cmd)
        for aspect in self.aspects.itervalues():
            if aspect.hasFunc(cmd):
                return aspect.call(cmd,*args,**kwargs)
                
    def tryCall(self, cmd, default, *args, **kwargs):
        for aspect in self.aspects.itervalues():
            if aspect.hasFunc(cmd):
                return aspect.call(cmd,*args,**kwargs)
        return default
        
    def callAll(self, cmd, *args, **kwargs):
        return [aspect.tryCall(cmd,None,*args,**kwargs) for aspect in self.aspects.itervalues()]
        
    def publish(self, cmd, *args, **kwargs):
        self.callAll('msg_%s' % cmd,*args,**kwargs)
        
        for child in self.children.itervalues():
            child.publish(cmd,*args,**kwargs)
            
        self.callAll('unmsg_%s' % cmd,*args,**kwargs)

    def attachToParent(self, parent):
        if parent != self.parent:
            self.detachFromParent()
            self.parent = parent
            if self.parent:
                self.parent.addChild(self)
                self.updatePath()
                self.callAll("updatePath")
                self.callAll('on_attach')
                
    def detachFromParent(self):
        if self.parent:
            parent = self.parent
            self.parent = None
            parent.removeChild(self)
            self.updatePath()
            self.callAll("updatePath")
            self.callAll('on_detach')
            
    def hasChild(self, name):
        return name in self.children
        
    def getChild(self, name):
        return self.children.get(name)
        
    def getParent(self):
        return self.parent
        
    def getEntity(self, path):
        if path == "/":
            return self.getRoot()
        elif path.startswith("/"):
            return self.getRoot().getEntity(path[1:])
        else:
            def getLocalEntity(name):
                if name == ".":
                    return self
                elif name == "..":
                    assert self.parent
                    return self.parent
                else:
                    assert self.hasChild(name)
                    return self.getChild(name)
            pos = path.find("/")
            if pos == -1:
                return getLocalEntity(path)
            else:
                return getLocalEntity(path[:pos]).getEntity(path[pos+1:])
                
    def enumerateAncestors(self, includeSelf = False):
        if includeSelf:
            ent = self
        else:
            ent = self.parent
        while ent:
            yield ent
            ent = ent.parent
            
    def filterAncestors(self, cond, includeSelf = False):
        return filter(cond,self.enumerateAncestors(includeSelf))
        
    def searchAncestors(self, cond, includeSelf = False):
        for ent in self.enumerateAncestors(includeSelf):
            if cond(ent):
                return ent
                
    def enumerateDescendents(self, includeSelf = False):
        if not includeSelf:
            return sum([child.enumerateDescendents(True) for child in self.children.itervalues()],[])
        else:
            return sum([child.enumerateDescendents(True) for child in self.children.itervalues()],[self])
            
    def filterDescendents(self, cond, includeSelf = False, deep = False):
        if not includeSelf:
            return sum([child.enumerateDescendents(True,deep) for child in self.children.itervalues()],[])
        else:
            r = []
            t = cond(self)
            if t:
                r.append(self)
            if not t or deep:
                map(r.extend,[child.filterDescendents(True,deep) for child in self.children.itervalues()])
            return r
        
    def searchDescendents(self, cond, includeSelf = False):
        for ent in self.enumerateDescendents(includeSelf):
            if cond(ent):
                return ent
        
    def getRoot(self):
        if self.parent:
            return self.parent.getRoot()
        else:
            return self
            
    def addChild(self, child):
        if child.name not in self.children:
            self.children[child.name] = child
            child.attachToParent(self)
            
    def removeChild(self, child):
        if child.name in self.children:
            self.children.pop(child.name).detachFromParent()
            
    def setVar(self, name, val):
        self.vars[name] = val
        self.callAll('set_%s' % name, val)
        
    def getVarNames(self):
        names = self.vars.keys()
        for aspect in self.aspects.itervalues():
            for name in aspect.funcs.iterkeys():
                if name.startswith('get_') and name[4:] not in names:
                    names.append(name[4:])
        return names
        
    def getVar(self, name, default = None):
        val = self.tryCall('get_%s' % name,None) or self.vars.get(name,default)
        if callable(val):
            return val()
        else:
            return val
        
    def hasVar(self, name):
        return name in self.vars
        
    def addAspect(self, aspect):
        if not self.hasAspect(aspect.name):
            self.aspects[aspect.name] = aspect
            aspect.attachToEntity(self)
            
    def removeAspect(self, aspect):
        if self.hasAspect(aspect.name):
            self.aspects.pop(aspect.name).detachFromEntity()
            
    def buildAspect(self, name):
        if not self.hasAspect(name):
            aspect = Util.dynload(name)()
            aspect.init(name,self)
            self.aspects[name] = aspect
        return self.aspects[name]
        
    def hasAspect(self, name):
        return name in self.aspects
        
    def getAspect(self, name):
        pos = name.find(':')
        if pos == -1:
            return self.aspects.get(name)
        else:
            return self.getEntity(name[:pos]).getAspect(name[pos+1:])
        
    def log(self, *v):
        print "%s: %s" % (self.path, ' '.join(map(str,v)))
