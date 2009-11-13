
from FuncMap import FuncMap

class Aspect(FuncMap):
    def __init__(self, **args):
        FuncMap.__init__(self)
        
        self.funcs = dict([(name,getattr(self,name)) for name in dir(self) if callable(getattr(self,name))])
        
        self.entity = None
        
        self.name = args.get('name','Aspect')
        
        self.attachToEntity(args.get('entity',None))
        
        for key, val in args.iteritems():
            self.call("load_%s" % key,val)
        
    def attachToEntity(self, entity):
        if entity != self.entity:
            self.detachFromEntity()
            self.entity = entity
            if self.entity:
                self.entity.addAspect(self)
                self.call("on_attach")
                
    def detachFromEntity(self):
        if self.entity:
            entity = self.entity
            self.entity = None
            entity.removeAspect(self)
            self.call("on_detach")
            
    def getName(self):
        return self.name
        
    def getPath(self):
        return (self.entity.getPath() if self.entity else "<no entity>") + "." + self.getName()

    def log(self, *v):
        print "%s: %s" % (self.getPath(),' '.join(map(str,v)))
        
    def hasFunc(self, name):
        return name in self.funcs
        
    def call(self, name, *args, **kwargs):
        return self.funcs[name](*args,**kwargs) if self.hasFunc(name) else None
        
    def getFuncNames(self):
        return self.funcs.keys()
        
    def doSave(self):
        return dict([(name[len("save_"):],func()) for name, func in self.funcs.iteritems() if name.startswith("save_")])
