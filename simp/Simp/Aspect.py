# -*- coding: utf-8 -*-

import Util
from FuncMap import FuncMap

class Aspect(FuncMap):
    def __init__(self, **args):
        FuncMap.__init__(self)
        
        self.name = args['name']
        self.entity = None
        
        self.attachToEntity(args.get('entity',None))
        
        for name, val in args.iteritems():
            self.call('load_%s' % name,val)
        
    def attachToEntity(self, entity):
        if self.entity != entity:
            self.detachFromEntity()
            self.entity = entity
            if self.entity:
                self.entity.addAspect(self)
                self.call('on_attach')
                
    def detachFromEntity(self):
        if self.entity:
            entity = self.entity
            self.entity = None
            entity.removeAspect(self)
            self.call('on_detach')
            
    def serialize(self):
        return dict([(name[len('save_'):],self.call(name)) for name in self.funcs.iterkeys() if name.startswith('save_')])
