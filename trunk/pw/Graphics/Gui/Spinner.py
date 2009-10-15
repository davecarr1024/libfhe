# -*- coding: utf-8 -*-

from Graphics.Prims2.SceneNode import SceneNode
from Graphics.Prims2.Rect import Rect
from Graphics.Prims2.Text import Text
from Graphics.Gui.Button import Button
from Core.Math.Vec2 import Vec2

class Spinner(SceneNode):
    def onAttach(self):
        SceneNode.onAttach(self)
        
        fill = lambda: self.getVar("fill",dict(color = (0,0,0)))
        stroke = lambda: self.getVar("stroke",dict(color = (0.5,0.5,0.5)))
        
        buttonSize = self.getVar("buttonSize",0.25)
        self.leftButton = Button(parent = self, name = "leftButton", vars = dict(fill = fill, 
                                                                                 stroke = stroke,
                                                                                 text = lambda: self.getVar("leftText","<"),
                                                                                 pos = Vec2(0,0),
                                                                                 scale = Vec2(buttonSize,1)))
        self.rightButton = Button(parent = self, name = "rightButton", vars = dict(fill = fill,
                                                                                   stroke = stroke,
                                                                                   text = lambda: self.getVar("rightText",">"),
                                                                                   pos = Vec2(1-buttonSize,0),
                                                                                   scale = Vec2(buttonSize,1)))
        self.centerRect = Rect(parent = self, name = "centerRect", vars = dict(material = fill,
                                                                               pos = Vec2(buttonSize,0),
                                                                               scale = Vec2(1-buttonSize*2,1)))
        Rect(parent = self.centerRect, name = "centerRectEdge", vars = dict(material = stroke,
                                                                            filled = False))
        self.centerText = Text(parent = self.centerRect, name = "centerText", 
            vars = dict(material = stroke,
                        align = 'center',
                        pos = Vec2(0.5,0),
                        text = lambda: str(self.getVar("selection",""))))
                        
        self.setVar("selection",lambda: self.getVar("items",[])[self.getVar("selectionIndex",0)])
                        
    def msg_buttonClick(self, button):
        if button == self.leftButton:
            self.inc(-1)
        elif button == self.rightButton:
            self.inc(1)
            
    def inc(self, d):
        self.setVar("selectionIndex",(self.getVar("selectionIndex",0) + d) % len(self.getVar("items",[])))
