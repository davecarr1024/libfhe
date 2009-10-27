# -*- coding: utf-8 -*-
@self.func(int,int)
def foo(i):
    return i * 2

assert(self.foo(3) == 6)

entity.setVar("b",True)
assert entity.getVar("b") == True

@self.func(None,'Var')
def set_setTest(val):
    assert isinstance(val,str)
    entity.setVar("setTestVal",val)
    
entity.setVar("setTest","hello")
assert entity.getVar("setTestVal") == "hello"

@self.func('Var',None)
def get_getTest():
    return "got"
    
assert entity.getVar("getTest") == "got"

@self.func(None,'VarMap')
def msg_msgTest(args):
    assert isinstance(args,dict)
    entity.setVar("msgTestVal",args['msg'])
    
entity.publish('msgTest',dict(msg='test'))
assert entity.getVar("msgTestVal") == "test"

@self.func(int,None)
def foo():
    return -1
    
assert entity.foo() == -1
assert self.foo() == -1

assert self.getName() == "TestAspect"
assert self.getPath() == "/.TestAspect"

entity.s = "what"
assert entity.s == "what"

child = entity.loadChild("test.xml")
assert child
assert child.name == "fileEnt"

assert child.i == 23

gchild = child.getChild("gchild")
assert gchild
assert gchild.s == "hello"

assert gchild.foo() == "bar"

gchild.getRoot().setVar("getRootTest",True)
assert entity.getVar("getRootTest",False) == True
assert gchild.getRoot().getVar("getRootTest",False) == True

assert Vec3
