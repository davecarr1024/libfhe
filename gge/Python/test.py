# -*- coding: utf-8 -*-

self.i = 1
assert self.i == 1 

assert self.hasAspect("Python/Script")

assert not self.hasAspect("Test/TestAspect")
self.addAspect("Test/TestAspect")
assert self.hasAspect("Test/TestAspect")
self.removeAspect("Test/TestAspect")
assert not self.hasAspect("Test/TestAspect")

e = self.app.getEntity(self.name)
e.sameCheck = True
assert self.sameCheck

self.runScript("Python/testFuncs.py")

self.classFunc("class")
assert self.classFuncVal == "class"

self.baseFunc("base")
assert self.baseFuncVal == "base"
