
class FuncMap:
    def __init__(self):
        
    def hasFunc(self, name):
        return name in self.funcs
        
    def call(self, name, arg = None):
        return self.funcs[name](arg) if self.hasFunc(name) else None
