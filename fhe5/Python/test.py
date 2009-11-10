# -*- coding: utf-8 -*-

self.a = 1
assert self.a == 1

assert not self.hasFunc("foo")
self.runScript("Python/testFuncs.py")
assert self.hasFunc("foo")
assert self.foo() == "bar"
