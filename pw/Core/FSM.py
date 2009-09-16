# -*- coding: utf-8 -*-
from Node import Node
import Util

class FSM(Node):
    def __init__(self, **args):
        Node.__init__(self, **args)
        
        self.stateStack = []
        
        self.transition(self.getVar("startState","start"))
        
    def pushState(self, state):
        self.stateStack.push(self.getVar("state",None))
        self.transition(state)
        
    def popState(self):
        assert self.stateStack
        self.transition(self.stateStack.pop())
    
    def transition(self, state):
        lastState = self.getVar("state",None)
        if lastState != state:
            self.call("exit_%s" % lastState)
            self.setVar("state",state)
            self.call("enter_%s" % state)
                
    def publish(self, cmd, *args, **kwargs):
        stateCmd = "%s_%s" % (self.getVar("state",None),cmd)
        
        self.call("msg_%s" % stateCmd, *args,**kwargs)
            
        for child in self.children.values():
            child.publish(cmd,*args,**kwargs)
            
        self.call("unmsg_%s" % stateCmd, *args,**kwargs)
