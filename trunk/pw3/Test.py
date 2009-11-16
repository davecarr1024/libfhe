#!/usr/bin/env python

from Core.Entity import Entity

root = Entity()

ent = root.buildChild(name = "ent")

assert ent.getName() == "ent"
assert ent.getPath() == "/ent"
assert ent.getParent() == root
assert ent.getRoot() == root
assert root.hasChild("ent")
assert root.getChild("ent") == ent

child = ent.buildChild(name = "child")
assert child.getParent() == ent
assert child.getRoot() == root

assert child.getEntity("/") == root
assert child.getEntity("..") == ent
assert child.getEntity("../../ent") == ent
assert child.getEntity(".") == child
assert child.getEntity("/ent/child") == child

child.buildAspect("Test.TestAspect")
assert child.call("foo") == "bar"
root.publish("msgTest","hello")
assert child.getVar("msgTestVal") == "hello"
child.setVar("setTest","world")
assert child.getVar("setTestVal") == "world"
assert child.getVar("getTest") == "got"

fileEnt = child.loadChild("fileEnt","Test.app")

assert fileEnt.getVar("b") == False
assert fileEnt.getVar("i") == 23
assert fileEnt.getVar("f") == 2.5
assert fileEnt.getVar("s") == "what!"
assert fileEnt.hasChild("fileChild")
fileChild = fileEnt.getChild("fileChild")
assert fileChild
assert fileChild.call("foo") == "bar"

from Core.Math.Mat3 import Mat3
Mat3.test()

from Core.Math.Quat import Quat
Quat.test()

from Core.Math.Mat4 import Mat4
Mat4.test()
