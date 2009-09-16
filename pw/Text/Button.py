from Widget import Widget
from Label import Label

class Button(Widget):
    def __init__(self, **args):
        Widget.__init__(self, **args)
        
        Label(parent = self, vars = dict(text = lambda: "[ %s ]" % self.getVar("text",""),
                                         fg = lambda: self.getVar("fg","white"),
                                         bg = lambda: self.getVar("bg","black"),
                                         align = lambda: self.getVar("align","left")))
                                         
    def msg_textInput(self, ch):
        if self.getVar("focused",False) and ch == self.getVar("acceptChar",10):
            self.globalPublish("acceptButton",self)
