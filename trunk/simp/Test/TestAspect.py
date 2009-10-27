# -*- coding: utf-8 -*-

from Simp.Aspect import Aspect

class TestAspect(Aspect):
    def double(self, i):
        return i * 2
        
    def save_foo(self):
        return "bar!"
