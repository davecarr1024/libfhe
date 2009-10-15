# -*- coding: utf-8 -*-
import Util

from Math.Vec2 import Vec2
from Math.Vec3 import Vec3
from Math.Quat import Quat
import math

class Node:
    nameCounts = {}
    logFile = open("pw.log","w")
    
    def __init__(self, **data):
        name = data.pop('name',self.__class__.__module__)
        Node.nameCounts[name] = Node.nameCounts.get(name,0) + 1
        if Node.nameCounts[name] == 1:
            self.name = name
        else:
            self.name = "%s_%d" % (name,Node.nameCounts[name])
        
        self.parent = None
        
        self.children = {}
        self.hasChild = self.children.__contains__
        self.getChild = self.children.get
        
        self.vars = {}
        #self.setVar = self.vars.__setitem__
        self.hasVar = self.vars.__contains__
        self.removeVar = self.vars.pop
        self.defaultVar = self.vars.setdefault
        
        self.funcs = dict([(name,getattr(self,name)) for name in dir(self) if callable(getattr(self,name))])
        self.hasFunc = self.funcs.__contains__
        
        self.attachToParent(data.pop('parent',None))
        
        self.doLoad(**data)
        
    #loading
        
    @staticmethod
    def buildNode(**data):
        nodeType = data.pop('type','Core.Node')
        nodeClass = Util.dynload(nodeType)
        assert nodeClass, "Unable to load class for node type %s" % nodeType
        return nodeClass(**data)
    
    @staticmethod
    def load(filename):
        namespace = dict(Vec2 = Vec2, Vec3 = Vec3, Quat = Quat, math = math)
        return Node.buildNode(**Util.evalLoad(filename,namespace))
    
    def doLoad(self, **data):
        for key, val in data.iteritems():
            fname = "load_%s" % key
            assert self.hasFunc(fname), "Unknown file tag %s" % key
            self.call(fname,val)
            
    def load_vars(self, data):
        for name, val in data.iteritems():
            self.setVar(name,val)
        
    def load_include(self, data):
        for subData in map(Util.evalLoad,Util.flatten(data)):
            self.doLoad(**subData)
            
    def load_children(self, data):
        for childName, childData in data.iteritems():
            childData['name'] = childName
            self.addChild(Node.buildNode(**childData))
            
    #saving
            
    def save(self, filename):
        Util.save(filename,self.doSave())
        
    def doSave(self):
        return dict([(name[len("save_"):], func()) for (name, func) in self.funcs.iteritems() if name.startswith("save_")])
            
    def save_vars(self):
        return dict([(name, self.getVar(name)) for name in self.vars.iterkeys()])
    
    def save_children(self):
        return dict([(name,child.doSave()) for (name, child) in self.children.iteritems()])
    
    def save_type(self):
        return self.__class__.__module__
        
    def save_name(self):
        return self.name
    
    #vars
    
    def setVar(self, name, val):
        self.vars[name] = val
        self.call("set_%s" % name, val)
    
    def getVar(self, name, default = None):
        val = self.vars.get(name,default)
        if callable(val):
            return val()
        elif isinstance(val,str) and val and val[0] == "=":
            return Util.deepEval(val[1:],dict(self = self, Vec2 = Vec2, Vec3 = Vec3, Quat = Quat, math = math))
        else:
            return val
        
    #modify tree
    
    def attachToParent(self, parent):
        if parent != self.parent:
            assert not parent.hasAncestor(self)
            self.detachFromParent()
            self.parent = parent
            if self.parent:
                self.parent.addChild(self)
                self.onAttach()
                
    def detachFromParent(self):
        if self.parent:
            self.onDetach()
            parent = self.parent
            self.parent = None
            parent.removeChild(self)
            
    def addChild(self, child):
        if child.name not in self.children:
            assert not self.hasAncestor(child)
            self.children[child.name] = child
            child.attachToParent(self)
            
    def removeChild(self, child):
        if child.name in self.children:
            self.children.pop(child.name).detachFromParent()
            
    def onAttach(self):
        pass
    
    def onDetach(self):
        pass
            
    #inspect tree
    
    def __repr__(self):
        return "<Node %s>" % self.getPath()
    
    def getPath(self):
        if not self.parent:
            return "/"
        elif not self.parent.parent:
            return "/" + self.name
        else:
            return self.parent.getPath() + "/" + self.name
            
    def getRoot(self):
        if self.parent:
            return self.parent.getRoot()
        else:
            return self
        
    def getNode(self, path):
        if path == "/":
            return self.getRoot()
        elif path.startswith("/"):
            return self.getRoot().getNode(path[1:])
        else:
            def getRelNode(relPath):
                if relPath == ".":
                    return self
                elif relPath == "..":
                    assert self.parent, "Path underflow"
                    return self.parent
                else:
                    child = self.getChild(relPath)
                    assert child, "No child named %s" % relPath
                    return child
                
            pos = path.find("/")
            if pos == -1:
                return getRelNode(path)
            else:
                return getRelNode(path[:pos]).getNode(path[pos+1:])
            
    def enumerateAncestors(self, includeSelf = False):
        if includeSelf:
            node = self
        else:
            node = self.parent
        while node:
            yield node
            node = node.parent
            
    def searchAncestors(self, cond, includeSelf = False):
        for node in self.enumerateAncestors(includeSelf):
            if cond(node):
                return node
            
    def filterAncestors(self, cond, includeSelf = False):
        return filter(cond,self.enumerateAncestors(includeSelf))
    
    def enumerateDescendents(self):
        children = self.children.values()
        return sum([child.enumerateDescendents() for child in children],children)
        
    def searchDescendents(self, cond):
        for child in self.children.itervalues():
            if cond(child):
                return child
            
        for child in self.children.itervalues():
            val = child.searchDescendents(cond)
            if val:
                return val
            
    def filterDescendents(self, cond, deep = False):
        if deep:
            return filter(cond,self.enumerateDescendents())
        else:
            nodes = []
            for child in self.children.itervalues():
                val = cond(child)
                if val:
                    nodes.append(child)
                if not val or deep:
                    nodes.extend(child.filterDescendents(cond,deep))
            return nodes
                
    def hasAncestor(self, node):
        if node == self:
            return True
        elif self.parent:
            return self.parent.hasAncestor(node)
        else:
            return False
    
    #messages
    
    def call(self, __funcName__, *args, **kwargs):
        if __funcName__ in self.funcs:
            return self.funcs[__funcName__](*args,**kwargs)
    
    def globalPublish(self, cmd, *args, **kwargs):
        self.getRoot().publish(cmd,*args,**kwargs)
        
    def publish(self, cmd, *args, **kwargs):
        self.call("msg_%s" % cmd, *args, **kwargs)
            
        for child in self.children.values():
            child.publish(cmd,*args,**kwargs)
            
        self.call("unmsg_%s" % cmd, *args, **kwargs)
            
    #logging
    
    def log(self, *v):
        s = "%s: %s" % (self.getPath(), ' '.join(map(str,v)))
        print s
        Node.logFile.write(s + "\n")
        Node.logFile.flush()

if __name__ == "__main__":
    root = Node.buildNode(name = "Root")

    assert not root.parent
    assert root.name == "Root"

    node = Node.buildNode(parent = root, name = "Node", vars = dict(a = 1))

    assert node.name == "Node"
    assert node.parent == root
    assert root.getChild("Node") == node
    assert node.path == "/Node"
    assert node.getNode(".") == node
    assert node.getNode("..") == root
    assert node.getNode("/") == root
    assert node.getNode("../Node") == node
    assert node.getVar("a") == 1

    node.setVar("b", lambda: node.getVar('a') * 2 )
    assert node.getVar("b") == 2

    testNode = Node.load("Test/Test.node")
    root.addChild(testNode)
    assert testNode.name == "Node_2"
    assert testNode.getVar('d')['l'][2] == 'a'

    fileNode = testNode.getChild("fileNode")
    assert fileNode
    assert fileNode.getVar('x') == 11
