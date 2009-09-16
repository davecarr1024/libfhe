from graphics.prims2d.rect import Rect
from graphics.prims2d.text import Text
from core.vec2 import Vec2

class Button(Rect):
    def onAttach(self):
        self.defaultVar("fill",dict(color = (0,0,0)))
        self.defaultVar("stroke",dict(color = (0.5,0.5,0.5)))
        self.text = str(self.defaultVar("text",None))
        self.defaultVar("clickMsg","buttonClicked")
        self.material = self.fill
        
        Rect.onAttach(self)
        
        Text(parent = self,
             vars = dict(text = self.text,
                         material = self.stroke,
                         position = Vec2(0.5,0.5)))

        Rect(parent = self,
             vars = dict(material = self.stroke,
                         filled = False))
        
        self.clicking = False

    def msg_mouseButtonDown(self, args):
        if self in args['hitObjects']:
            self.clicking = True

    def msg_mouseButtonUp(self, args):
        if self in args['hitObjects'] and self.clicking:
            if self.clickMsg:
                self.publish(self.clickMsg)
        self.clicking = False
