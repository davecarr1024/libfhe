from SceneNode import SceneNode

class Widget(SceneNode):
    def __init__(self, **args):
        SceneNode.__init__(self, **args)
        
        def fg():
            if self.getVar("focused",False):
                return self.getVar("focus_fg","white")
            else:
                return self.getVar("unfocus_fg","white")
            
        self.setVar("fg",fg)
            
        def bg():
            if self.getVar("focused",False):
                return self.getVar("focus_bg","blue")
            else:
                return self.getVar("unfocus_bg","black")
            
        self.setVar("bg",bg)
    
    def msg_focus(self):
        self.setVar("focused",True)
        
    def msg_unfocus(self):
        self.setVar("focused",False)
        
    def onAttach(self):
        SceneNode.onAttach(self)
        self.getWindow().moveFocus(0)
        
    def onDetach(self):
        SceneNode.onDetach(self)
        self.getWindow().moveFocus(0)
