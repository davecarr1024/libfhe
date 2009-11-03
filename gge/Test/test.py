# -*- coding: utf-8 -*-

self.entity.i = 11
assert self.entity.i == 11

@self.func(None,None)
def noRetNoArg():
    self.entity.noRetNoArgVal = True
    
self.entity.noRetNoArg()
assert self.entity.noRetNoArgVal

@self.func(int,None)
def noArg():
    return 23
    
assert self.entity.noArg() == 23

@self.func(None,int)
def noRet(i):
    self.entity.noRetVal = i
    
self.entity.noRet(15)
assert self.entity.noRetVal == 15

@self.func('int','int')
def retAndArg(i):
    return i * i
    
assert self.entity.retAndArg(2) == 4

assert self.entity.app.hasEntity(self.entity.name)

@self.func(None,'Var')
def set_setTest(val):
    self.entity.setTestVal = val * 2
    
self.entity.setTest = 1
assert self.entity.setTestVal == 2

self.entity.setTest = "hi"
assert self.entity.setTestVal == "hihi"

@self.func('Var',None)
def get_getTest():
    return "got"
    
assert self.entity.getTest == "got"

@self.func(None,'VarMap')
def msg_pyMsgTest(args):
    self.entity.pyMsgTestVal = args['val']

self.entity.app.publish('pyMsgTest',dict(val = 'msg'))
assert self.entity.pyMsgTestVal == 'msg'

@self.func(int,None)
def crossTest():
    return -1
