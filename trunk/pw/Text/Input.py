from Widget import Widget
from Label import Label

class Input(Widget):
    def __init__(self, **args):
        Widget.__init__(self, **args)
        
        Label(parent = self, vars = dict(text = lambda: "%s %s" % (self.getVar("prompt",""), self.getVar("text","")),
                                         fg = lambda: self.getVar("fg","white"),
                                         bg = lambda: self.getVar("bg","black")))
        
    def msg_textInput(self, ch):
        if self.getVar("focused",False):
            if ch == self.getVar("acceptChar",10):
                self.globalPublish("acceptInput",self,self.getVar("text",""))
            elif ch == self.getVar("backspaceChar",263):
                self.setVar("text",self.getVar("text","")[:-1])
            elif ch >= 0 and ch < 256:
                self.setVar("text",self.getVar("text","") + chr(ch))
