from graphics.prims2d.rect import Rect
from graphics.prims2d.text import Text
from core.vec2 import Vec2

class TextBox(Rect):
    def onAttach(self):
        self.defaultVar("stroke",dict(color = (0.5,0.5,0.5)))
        self.defaultVar("highlightStroke",dict(color = (1,1,1)))
        self.defaultVar("fill",dict(color = (0,0,0)))
        self.defaultVar("text","")
        self.defaultVar("static",False)
        
        Rect.onAttach(self)
        
        self.edge = Rect(parent = self,
                         vars = dict(filled = False,
                                     material = self.stroke,
                                     static = False))
                                     
        self.textPrim = Text(parent = self,
                             vars = dict(text = self.text,
                                         material = self.stroke,
                                         static = False,
                                         position = Vec2(0.01,0.5),
                                         align = 'left'))
                                     
        self.active = False
        
    def msg_mouseButtonDown(self, args):
        if self in args['hitObjects']:
            self.active = True
            self.edge.material = self.highlightStroke
        else:
            self.active = False
            self.edge.material = self.stroke
            
    def addText(self, text):
        self.text += text
        self.textPrim.text += text
        
    def removeText(self, amount = 1):
        self.text = self.text[:-amount]
        self.textPrim.text = self.textPrim.text[:-amount]
        
    def clearText(self):
        self.text = ""
        self.textPrim.text = ""
            
    def msg_keyDown(self, args):
        if self.active:
            name = args['name']
            if name == 'backspace' and self.text:
                self.removeText()
            elif name == 'space':
                self.addText(' ')
            elif len(name) == 1:
                self.addText(name)
