# -*- coding: utf-8 -*-

from Graphics.Prims2.SceneNode import SceneNode
from Graphics.Prims2.Text import Text
from Graphics.Prims2.Rect import Rect

class Button(SceneNode):
    def onAttach(self):
        SceneNode.onAttach(self)
        
        fill = lambda: self.getVar("fill",dict(color = (0,0,0)))
        stroke = lambda: self.getVar("stroke",dict(color = (0.5,0.5,0.5)))
        
        self.bg = Rect(parent = self, name = "bg", vars = dict(material = fill))
        Rect(parent = self, name = "edge", vars = dict(material = stroke,
                                                       filled = False))
        Text(parent = self.bg, name = "text", vars = dict(material = stroke,
                                                          text = lambda: self.getVar("text")))
                                                       
    def msg_mouseButtonDown(self, **args):
        if args.get("buttonName") == "left" and self.bg in args.get("hitObjects",[]):
            self.globalPublish("buttonClick",self)
            self.getRoot().save("dump")
