from Widget import Widget
from Label import Label

class Spinner(Widget):
    def __init__(self, **args):
        Widget.__init__(self, **args)
        
        self.setVar("selectedItem", lambda: str(self.getVar("options",[])[self.getVar("selection",0)]))
        
        Label(parent = self,
              vars = dict(text = lambda: "< %s >" % self.getVar("selectedItem",""),
                          fg = lambda: self.getVar("fg","white"),
                          bg = lambda: self.getVar("bg","black")))
                          
    def msg_textInput(self, ch):
        if self.getVar("focused",False):
            if ch == self.getVar("acceptChar",10):
                self.globalPublish("acceptSpinner",self,self.getVar("selectedItem",""))
            elif ch == self.getVar("decSelectionChar",260):
                self.moveSelection(-1)
            elif ch == self.getVar("incSelectionChar",261):
                self.moveSelection(1)
                
    def moveSelection(self, d):
        self.setVar("selection",(self.getVar("selection",0) + d) % len(self.getVar("options",[])))
