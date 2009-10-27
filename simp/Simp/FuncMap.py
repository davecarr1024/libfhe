# -*- coding: utf-8 -*-

class FuncMap:
    def __init__(self):
        self.funcs = dict([(name,getattr(self,name)) for name in dir(self) if callable(getattr(self,name))])
        
    def hasFunc(self, name):
        return name in self.funcs
        
    def call(self, name, *args, **kwargs):
        if self.hasFunc(name):
            return self.funcs[name](*args,**kwargs)
