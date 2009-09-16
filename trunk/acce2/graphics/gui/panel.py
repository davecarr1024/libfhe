from graphics.prims2d.rect import Rect
from graphics.prims2d.text import Text
from graphics.prims2d.line import Line
from button import Button
from core.vec2 import Vec2

class Panel(Rect):
    def onAttach(self):
        self.defaultVar("stroke",dict(color = (0.5,0.5,0.5)))
        self.defaultVar("fill",dict(color = (0,0,0)))
        self.defaultVar("showTitleBar",True)
        self.defaultVar("titleStroke",self.stroke)
        self.defaultVar("titleFill",self.fill)
        self.defaultVar("title","")
        self.defaultVar("titleBarSize",0.1)
        self.defaultVar("static",False)
        self.defaultVar("canClose",True)
        self.defaultVar("canResize",True)
        self.defaultVar("resizeSize",0.05)

        self.material = self.fill

        Rect.onAttach(self)

        Rect(parent = self,
             vars = dict(material = self.stroke,
                         filled = False))

        if self.showTitleBar:
            self.titleBar = Rect(parent = self,
                                 vars = dict(material = self.titleFill,
                                             position = Vec2(0,-self.titleBarSize),
                                             scale = Vec2(1,self.titleBarSize)))
            Rect(parent = self.titleBar,
                 vars = dict(material = self.stroke,
                             filled = False))

            Text(parent = self.titleBar,
                 vars = dict(material = self.titleStroke,
                             text = self.title,
                             align = 'left',
                             position = Vec2(0.01,0.5)))
            
        else:
            self.titleBar = None

        if self.canClose and self.titleBar:
            self.closeButton = Button(parent = self.titleBar,
                                      vars = dict(fill = self.fill,
                                                  stroke = self.stroke,
                                                  position = Vec2(0.91,0.1),
                                                  scale = Vec2(0.08,0.8),
                                                  text = "X"))

            #Line(parent = self.closeButton,
                 #vars = dict(material = self.stroke,
                             #v1 = Vec2(0,0),
                             #v2 = Vec2(1,1)))
            #Line(parent = self.closeButton,
                 #vars = dict(material = self.stroke,
                             #v1 = Vec2(1,0),
                             #v2 = Vec2(0,1)))
            
        else:
            self.closeButton = None
            
        if self.canResize:
            self.resizeBlock = Rect(parent = self,
                                    vars = dict(filled = False,
                                                material = self.stroke,
                                                position = Vec2(1-self.resizeSize,1-self.resizeSize),
                                                scale = Vec2(self.resizeSize,self.resizeSize)))
        else:
            self.resizeBlock = None

        self.dragging = False
        self.resizing = False

    def msg_mouseButtonDown(self, args):
        localPos = self.globalToLocal(args['pos'])
        low = 1-self.resizeSize
        high = 1#1+self.resizeSize
        if self.canResize and localPos.x > low and localPos.x < high and localPos.y > low and localPos.y < high:
            self.resizeOffset = self.globalToParent(args['pos']) - self.position
            self.startScale = self.scale
            self.resizing = True
        elif self.titleBar in args['hitObjects']:
            self.dragOffset = localPos
            self.dragging = True

    def msg_mouseButtonUp(self, args):
        self.dragging = False
        self.resizing = False

    def msg_mouseMotion(self, args):
        if self.dragging:
            localMouse = self.globalToLocal(args['pos'])
            self.position = self.localToParent(localMouse - self.dragOffset)
        elif self.resizing:
            parentMouse = self.globalToParent(args['pos'])
            localMouse = parentMouse - self.position
            self.scale = self.startScale * (localMouse / self.resizeOffset)

    def msg_buttonClicked(self, args):
        if args['sender'] == self.closeButton:
            self.delete()
