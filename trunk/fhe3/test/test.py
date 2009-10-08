# -*- coding: utf-8 -*-
@self.func(int,int)
def foo(i):
    return i * 2

assert(self.foo(3) == 6)

e = self.getEntity()
assert e

e.setVar("b",True)
assert e.getVar("b") == True

@self.func(None,'Var')
def set_setTest(val):
    assert isinstance(val,str)
    e.setVar("setTestVal",val)
    
e.setVar("setTest","hello")
assert e.getVar("setTestVal") == "hello"

@self.func('Var',None)
def get_getTest():
    return "got"
    
assert e.getVar("getTest") == "got"

@self.func(None,'VarMap')
def msg_msgTest(args):
    assert isinstance(args,dict)
    e.setVar("msgTestVal",args['msg'])
    
e.publish('msgTest',dict(msg='test'))
assert e.getVar("msgTestVal") == "test"
