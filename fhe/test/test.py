# -*- coding: utf-8 -*-

import math

r = Rot(1)
assert r.angle == 1
r.angle = 2
assert r.angle == 2

assert Rot(3) == Rot(3)
assert Rot(1) + Rot(2) == Rot(3)
assert Rot(10) - Rot(5) == Rot(5)
assert Rot(3) * 4 == Rot(12)
assert Rot(4) / 2 == Rot(2)
assert Rot(math.pi/2) * Vec2(1,0) == Vec2(0,1)
assert Rot(math.pi/2 * 101).norm() == Rot(math.pi/2)

v = Vec2(1,2)
assert v.x == 1
assert v.y == 2
v.x = 3
v.y = 4
assert v.x == 3
assert v.y == 4

assert Vec2(5,6) == Vec2(5,6)

assert Vec2(10,20) * 2 == Vec2(20,40)
assert Vec2(100,-50) / 5 == Vec2(20,-10)
assert Vec2(-10,0) + Vec2(20,-5) == Vec2(10,-5)
assert Vec2(200,150) - Vec2(100,100) == Vec2(100,50)
assert Vec2(-1,0).dot(Vec2(1,0)) == -1
assert Vec2(3,4).length() == 5
assert Vec2(10,0).norm() == Vec2(1,0)

v = Vec2(1,2)
self.setVar("v2",v)
assert self.getVar("v2") == v

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

v = Vec2(1,2)
self.setVar("v2",v)
assert self.getVar("v2") == v

r = Rot(1)
self.setVar("r",r)
assert self.getVar("r") == r

v = Vec3(4,5,6)
self.setVar("v3",v)
assert self.getVar("v3") == v

q = Quat(Vec3(0,1,0),2)
self.setVar("q",q)
assert self.getVar("q") == q

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

@self.func(None,Var)
def set_setTest(val):
    self.setVar("setTestVal",val.get())
    
assert not self.hasVar("setTestVal")
self.setVar("setTest",11)
assert self.hasVar("setTestVal")
assert self.getVar("setTestVal") == 11

@self.func(Var,None)
def get_getTest():
    return Var("got")
    
assert self.getVar("getTest") == "got"
