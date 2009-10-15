# -*- coding: utf-8 -*-
from Core.Aspect import Aspect

class TestAspect(Aspect):
    def on_attach(self):
        assert self.entity.call('foo',-1) == -2
        
    def foo(self,a):
        return a * 2
