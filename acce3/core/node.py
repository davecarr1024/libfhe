from visitor import Visitor

class CallVisitor(Visitor):
  def __init__(self, name, *args, **kwargs):
    self.name = name
    self.args = args
    self.kwargs = kwargs
    
  def visit(self, node):
    return node.call(self.name, *self.args, **self.kwargs)
    
  def unvisit(self, node):
    node.call('un_%s' % self.name, *self.args, **self.kwargs)

class AggregateCallVisitor(CallVisitor):
  def __init__(self, name, *args, **kwargs):
    CallVisitor.__init__(self, name, *args, **kwargs)
    self.res = []
    
  def visit(self, node):
    res = CallVisitor.visit(self, node)
    if res != None:
      self.res.append(res)
    
class Node:
  def __init__(self, name, *children, **args):
    self.name = name
    self.children = children
    for name, val in args.iteritems():
      self[name] = val
    
  def __getitem__(self, name):
    return self.get(name)
    
  def __setitem__(self, name, val):
    self.set(name, val)
    
  def __contains__(self, name):
    return self.has(name)
    
  def get(self, name, default = None):
    return getattr(self, name, default)
    
  def set(self, name, val):
    setattr(self, name, val)
    
  def has(self, name):
    return hasattr(self, name)
    
  def visit(self, visitor):
    visitor.visit(self)
    for child in self.children:
      child.visit(visitor)
    visitor.unvisit(self)
    
  def call(self, name, *args, **kwargs):
    if name in self:
      return self[name](*args, **kwargs)
      
  def visit_call(self, name, *args, **kwargs):
    self.visit(CallVisitor(name, *args, **kwargs))

  def aggregate_visit_call(self, name, *args, **kwargs):
    visitor = AggregateCallVisitor(name, *args, **kwargs)
    self.visit(visitor)
    return visitor.res

