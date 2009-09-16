# -*- coding: utf-8 -*-

assert self.name == "pythonTest"
assert self.type == "TestNode"
assert self.path == "/"
assert not self.getParent()

assert self.call("square",3.0) == 9.0
assert self.call("noarg",None) == 4

assert not self.hasVar("b")
self.setVar("b",True)
assert self.hasVar("b")
assert self.getVar("b") == True

self.setVar("i",1)
assert self.getVar("i") == 1

self.setVar("f",3.0)
assert self.getVar("f") == 3.0

self.setVar("s","what")
assert self.getVar("s") == "what"

@self.func(None,str)
def hello(msg):
    self.setVar("hello_msg",msg)
    
@self.func(None,None)
def echo():
    self.setVar("didEcho",True)

@self.func(int,int)
def pySquare(i):
    return i * i

@self.func(str,None)
def getMsg():
    return "msg"

@self.func(None,str)
def msg_pyTest(s):
    self.setVar("pyTestVal",s)
    
@self.func(float,float)
def double(f):
    return f * 2

assert self.call("double",2.0) == 4.0

child = self.getChild("pychild")
assert child
assert child.getParent() == self
assert child.name == "pychild"
assert child.type == "Node"
assert child.path == "/pychild"

@child.func(None,None)
def childFunc():
    child.setVar("funcVal",10)

assert not child.hasVar("funcVal")
child.call("childFunc",None)
assert child.getVar("funcVal") == 10

assert child.getRoot() == self
assert child.getNode("..") == self
assert self.getNode("/pychild/.././pychild") == child
assert self.getNode(child.path) == child

child.detachFromParent()
assert not child.getParent()
assert child.getRoot() == child
assert not self.hasChild("pychild")

self.addChild(child)
assert self.hasChild("pychild")
assert child.getParent() == self

child.release()
assert not self.hasChild("pychild")

assert self.buildNode("Node","newnode")

assert not self.buildNode("","")

v = VarMap()
v["b"] = True
assert v["b"] == True
v["i"] = 1
assert v["i"] == 1
v["f"] = 2.0
assert v["f"] == 2.0
v["s"] = "hey"
assert v["s"] == "hey"
v["nest"] = VarMap( dict( nest = "wow" ) )
assert v["nest"]["nest"] == "wow"

self.setVar("vm",v)
_v = self.getVar("vm")
assert _v["b"] == True
assert _v["i"] == 1
assert _v["f"] == 2.0
assert _v["s"] == "hey"
assert _v["nest"]["nest"] == "wow"

@self.func(None,VarMap)
def vmTest(val):
    self.setVar("vmTest",val["val"])

self.call("vmTest", VarMap( dict( val = 11 ) ) )
assert self.getVar("vmTest") == 11

child = self.buildNode("Node","child")
assert child
child.attachToParent(self)
assert self.getChild(child.name) == child
assert child.getParent() == self

@child.func(None,int)
def msg_publishTest(i):
    child.setVar("i",i)

assert not child.hasVar("i")
self.publish("publishTest",1)
assert child.getVar("i") == 1
