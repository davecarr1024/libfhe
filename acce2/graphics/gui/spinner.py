from graphics.prims2d.rect import Rect
from graphics.prims2d.text import Text
from core.vec2 import Vec2
from image import Image
from button import Button

class Spinner(Rect):
    def onAttach(self):
        self.defaultVar("fill",dict(color = (0,0,0)))
        self.defaultVar("downFill",self.fill)
        self.defaultVar("downText","<")
        self.defaultVar("upText",">")
        self.defaultVar("upFill",self.fill)
        self.defaultVar("stroke",dict(color = (0.5,0.5,0.5)))
        self.options = map(str,self.defaultVar("options",[None]))
        self.defaultVar("static",False)
        
        self.material = self.fill
        
        Rect.onAttach(self)
        
        Rect(parent = self,
             vars = dict(filled = False,
                         material = self.stroke))

        self.down = Button(parent = self,
                           vars = dict(fill = self.downFill,
                                       stroke = self.stroke,
                                       text = self.downText,
                                       position = Vec2(0,0),
                                       scale = Vec2(0.25,1)))
                                       
        self.up = Button(parent = self,
                         vars = dict(fill = self.upFill,
                                     stroke = self.stroke,
                                     text = self.upText,
                                     position = Vec2(0.75,0),
                                     scale = Vec2(0.25,1)))
                                     
        self.text = Text(parent = self,
                         vars = dict(static = False,
                                     position = Vec2(0.5,0.5),
                                     material = self.stroke))
                                     
        self.setSelection(0)
        
    def setSelection(self, i):
        self.selection = i % len(self.options)
        self.text.text = self.options[self.selection]
        
    def nextSelection(self):
        self.setSelection(self.selection + 1)
        
    def lastSelection(self):
        self.setSelection(self.selection - 1)

    def getSelection(self):
        return self.options[self.selection]
        
    def msg_buttonClicked(self, args):
        if args['sender'] == self.up:
            self.nextSelection()
        elif args['sender'] == self.down:
            self.lastSelection()
