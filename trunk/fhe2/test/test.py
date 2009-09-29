# -*- coding: utf-8 -*-

self.setVar("b",True)
assert self.getVar("b") == True

self.setVar("i",11)
assert self.getVar("i") == 11

self.setVar("f",2)
assert self.getVar("f") == 2

self.setVar("s","hello")
assert self.getVar("s") == "hello"

self.setVar("v2",Vec2(1,2))
assert self.getVar("v2") == Vec2(1,2)

self.setVar("v3",Vec3(3,4,5))
assert self.getVar("v3") == Vec3(3,4,5)

self.setVar("r",Rot(-1))
assert self.getVar("r") == Rot(-1)

self.setVar("q",Quat(Vec3(0,1,0),1))
assert self.getVar("q") == Quat(Vec3(0,1,0),1)

@self.func(None,None)
def noArgOrRet():
    self.setVar("noArgOrRet",True)

self.call("noArgOrRet",None)
assert self.getVar("noArgOrRet") == True

@self.func(None,str)
def noRet(s):
    self.setVar("noRet",s)
    
self.call("noRet","arg")
assert self.getVar("noRet") == "arg"

@self.func(str,None)
def noArg():
    return "ret"
    
assert self.call("noArg",None) == "ret"

@self.func(str,str)
def argAndRet(s):
    return s * 2
    
assert self.call("argAndRet","arg") == "argarg"

@self.func(None,Var)
def set_setTest(v):
    self.setVar("setTestVal",v.get() * 2)

self.setVar("setTest",1)
assert self.getVar("setTestVal") == 2

@self.func(Var,None)
def get_getTest():
    return Var("got")
    
assert self.getVar("getTest") == "got"

child = self.buildChild("Node","child")
assert child
assert child.getParent() == self

@self.func(None,VarMap)
def vmTest(vm):
    self.setVar("vmVal",vm.getVar("val"))
    
self.call("vmTest",VarMap(dict(val = 10)))
assert self.getVar("vmVal") == 10

@child.func(None,VarMap)
def msg_msgTest(args):
    child.setVar("msgTest",args.getVar("msgTest"))
    
self.publish("msgTest",VarMap(dict(msgTest = 21)))
assert child.getVar("msgTest") == 21

fileChild = self.loadChild("test.xml")
assert fileChild

assert fileChild.getVar("b") == True
assert fileChild.getVar("i") == 11
assert fileChild.getVar("f") == 2.5
assert fileChild.getVar("s") == "hi"
assert fileChild.getVar("v2") == Vec2(1,2)
assert fileChild.getVar("v3") == Vec3(3,4,5)
assert fileChild.getVar("r") == Rot(-1)
assert fileChild.getVar("q") == Quat(Vec3(0,1,0),1)

@self.func(None,VarMap)
def msg_pub(args):
    self.setVar("pub","sub")
