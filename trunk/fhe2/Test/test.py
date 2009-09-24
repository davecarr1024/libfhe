# -*- coding: utf-8 -*-

self.setVar("b",True)
assert self.getVar("b") == True

self.setVar("i",11)
assert self.getVar("i") == 11

self.setVar("f",2)
assert self.getVar("f") == 2

self.setVar("s","hello")
assert self.getVar("s") == "hello"

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

child = self.buildNode("Node","child")
assert child
self.addChild(child)
assert child.getParent() == self

@child.func(None,VarMap)
def msg_msgTest(args):
    child.setVar("msgTest",args.getVar("msgTest"))
    
self.publish("msgTest",VarMap(dict(msgTest = 21)))
assert child.getVar("msgTest") == 21
