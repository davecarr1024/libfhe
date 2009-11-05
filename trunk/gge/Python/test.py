# -*- coding: utf-8 -*-

self.i = 1
assert self.i == 1 

assert self.hasAspect("Python/Script")

assert not self.hasAspect("Test/TestAspect")
self.addAspect("Test/TestAspect")
assert self.hasAspect("Test/TestAspect")
self.removeAspect("Test/TestAspect")
assert not self.hasAspect("Test/TestAspect")

self.runScript("Python/testFuncs.py")

self.classFunc("class")
assert self.classFuncVal == "class"

self.baseFunc("base")
assert self.baseFuncVal == "base"

self.d = dict(i = 1, s = "what", v = Vec2(1,2))
assert self.d["i"] == 1
assert self.d["s"] == "what"
assert self.d["v"].y == 2

self.v3 = Vec3(1,2,3)
assert self.v3.z == 3
