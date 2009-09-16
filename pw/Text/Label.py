from SceneNode import SceneNode

class Label(SceneNode):
    def __init__(self, **args):
        SceneNode.__init__(self, **args)
        
        self.setVar("box",lambda: (0,0,len(self.getVar("text","")),1))
        
    def msg_textRender(self, context):
        x,y = self.getVar("globalPos")
        text = self.getVar("text","")
        align = self.getVar("align","left")
        layer = self.getVar("layer",0)
        fg = self.getVar("fg","white")
        bg = self.getVar("bg","black")
        
        dx = 0
        if align == "center":
            dx = len(text)/-2
        elif align == "right":
            dx = -len(text)
            
        for i, c in enumerate(text):
            context.add(x+dx+i,y,layer,c,fg,bg)
