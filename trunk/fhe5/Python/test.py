# -*- coding: utf-8 -*-

self.a = 1
assert self.a == 1

assert not self.hasFunc("foo")
self.runScript("Python/testFuncs.py")
assert self.hasFunc("foo")
assert self.foo() == "bar"

@self.func
def msg_msgTest(arg):
    self.msgTestVal = arg
    
self.getRoot().publish("msgTest",12)
assert self.msgTestVal == 12

@self.func
def get_getTest(arg):
    return "got"
    
assert self.getTest == "got"

@self.func
def set_setTest(arg):
    self.setTestVal = arg
    
self.setTest = "hello"
assert self.setTestVal == "hello"

self.l = [1,2.5,"what"]
assert self.l[0] == 1
assert self.l[1] == 2.5
assert self.l[2] == "what"

self.d = dict(a = 2)
assert self.d["a"] == 2

self.v2 = Vec2(4,5)
assert self.v2 == Vec2(4,5)

self.r = Rot(5)
assert self.r == Rot(5)

self.v3 = Vec3(5,6,7)
assert self.v3 == Vec3(5,6,7)

self.q = Quat(Vec3(0,1,0),1)
assert self.q == Quat(Vec3(0,1,0),1)
