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

