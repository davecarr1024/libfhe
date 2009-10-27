# -*- coding: utf-8 -*-

import Util
from FuncMap import FuncMap

class System(FuncMap):
    def __init__(self, **args):
        FuncMap.__init__(self)
        
        self.name = args['name']
        self.sim = None
        
        self.attachToSim(args.get('sim',None))
        
        for name, val in args.iteritems():
            self.call('load_%s' % name,val)
        
    def attachToSim(self, sim):
        if sim != self.sim:
            self.detachFromSim()
            self.sim = sim
            if self.sim:
                self.sim.addSystem(self)
                self.call('on_attach')
                
    def detachFromSim(self):
        if self.sim:
            sim = self.sim
            self.sim = None
            sim.removeSystem(self)
            self.call('on_detach')

    def serialize(self):
        return dict([(name[len('save_'):],self.call(name)) for name in self.funcs.iterkeys() if name.startswith('save_')])
