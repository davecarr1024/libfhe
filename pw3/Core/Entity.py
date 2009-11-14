import Util

class Entity:
    def __init__(self, **args):
        self.name = args.get('name','Entity')
        self.parent = None
        self.vars = {}
        self.aspects = {}
        self.children = {}
        
        self.attachToParent(args.get('parent',None))
        
        for name, val in args.get('vars',{}).iteritems():
            self.setVar(name,val)
            
        for name, val in args.get('aspects',{}).iteritems():
            self.buildAspect(name,**val)
            
        for name, val in args.get('vars',{}).iteritems():
            self.setVar(name,val)
            
        for name, val in args.get('children',{}).iteritems():
            self.buildChild(name,**val)
            
    @staticmethod
    def load(filename):
        return Entity(**Util.deepLoad(filename))
        
    def doSave(self):
        return dict(vars = dict([(name,self.getVar(name)) for name in self.getVarNames()]),
                    aspects = dict([(name,aspect.doSave()) for name, aspect in self.aspects.iteritems()]),
                    children = dict([(name,child.doSave()) for name, child in self.children.iteritems()]))
        
    def save(self, filename):
        Util.save(filename,self.doSave())
            
    def attachToParent(self, parent):
        if parent != self.parent:
            self.detachFromParent()
            
            self.parent = parent
            if self.parent:
                self.parent.addChild(self)
                self.call("on_attach")
                
    def detachFromParent(self, parent):
        if self.parent:
            parent = self.parent
            self.parent = None
            parent.removeChild(self)
            self.call("on_detach")
            
    def addChild(self, child):
        if not self.hasChild(child.name):
            self.children[child.name] = child
            child.attachToParent(self)
            
    def removeChild(self, child):
        if self.hasChild(child.name):
            self.children.pop(child.name).detachFromParent()
            
    def buildChild(self, name, **args):
        args['name'] = name
        args['parent'] = self
        child = Entity(**args)
        self.addChild(child)
        return child
        
    def hasChild(self, name):
        return name in self.children
        
    def getChild(self, name):
        return self.children.get(name)
        
    def getParent(self):
        return self.parent
        
    def getRoot(self):
        ent = self.parent
        while ent:
            ent = ent.parent
        return ent
        
    def getEntity(self, path):
        if path == '/':
            return self.getRoot()
        elif path.startswith('/'):
            return self.getRoot().getEntity(path[1:])
        else:
            def getLocal(path):
                if path == ".":
                    return self
                elif path == "..":
                    assert self.parent
                    return self.parent
                else:
                    assert self.hasChild(path)
                    return self.getChild(path)
            pos = path.find('/')
            if pos == -1:
                return getLocal(path)
            else:
                return getLocal(path[:pos]).getEntity(path[pos+1:])
        
    def addAspect(self, aspect):
        if not self.hasAspect(aspect.name):
            self.aspects[aspect.name] = aspect
            aspect.attachToEntity(self)
            
    def removeAspect(self, aspect):
        if self.hasAspect(aspect.name):
            self.aspects.pop(aspect.name).detachFromEntity()
            
    def buildAspect(self, name, **args):
        args['name'] = name
        args['entity'] = self
        aspect = Util.dynload(name)(**args)
        self.addAspect(aspect)
        return aspect
        
    def hasAspect(self, name):
        return name in self.aspects
        
    def getAspect(self, name):
        return self.aspects.get(name)
        
    def getName(self):
        return self.name
        
    def getPath(self):
        if self.parent and self.parent.parent:
            return self.parent.getPath() + "/" + self.getName()
        elif self.parent:
            return "/" + self.getName()
        else:
            return "/"
            
    def log(self, *v):
        print "%s: %s" % (self.getPath(),' '.join(map(str,v)))
        
    def hasVar(self, name):
        return name in self.vars
        
    def getVar(self, name, default = None):
        return self.call("get_%s" % name) or self.vars.get(name,default)
        
    def setVar(self, name, val):
        self.vars[name] = val
        self.call("set_%s" % name,val)
        
    def defaultVar(self, name, val):
        self.setVar(name,self.getVar(name,val))
        return self.getVar(name)
        
    def getVarNames(self):
        names = self.vars.keys()
        for aspect in self.aspects.itervalues():
            for func in aspect.getFuncNames():
                if func.startswith("get_"):
                    names.append(func[len("get_"):])
        return names
        
    def hasFunc(self, name):
        for aspect in self.aspects.itervalues():
            if aspect.hasFunc(name):
                return True
        return False
        
    def call(self, name, *args, **kwargs):
        ret = None
        for aspect in self.aspects.itervalues():
            if aspect.hasFunc(name):
                ret = aspect.call(name,*args,**kwargs)
        return ret
        
    def publish(self, name, *args, **kwargs):
        self.call("msg_%s" % name,*ar)
        
        for child in self.children.itervalues():
            child.publish(name,arg)
            
        self.call("unmsg_%s" % name,arg)
