# -*- coding: utf-8 -*-

from Graphics.Prims2.SceneNode import SceneNode
from Graphics.Prims2.Text import Text
from Graphics.Prims2.Rect import Rect

class TextBox(SceneNode):
    def onAttach(self):
        SceneNode.onAttach(self)
        
        fill = lambda: self.getVar("fill",dict(color=(0,0,0)))
        stroke = lambda: self.getVar("stroke",dict(color=(0.5,0.5,0.5)))
        
        self.bg = Rect(parent = self, name = "bg", vars = dict(material=fill))
        Rect(parent = self, name = "edge", vars = dict(material = stroke, filled = False ) )
        self.text = Text(parent = self.bg, name = "text", vars = dict(material = stroke, text = lambda: self.getVar("text","")))
        
    def msg_keyDown(self, **args):
        name = args['name']
        if name == 'return':
            self.log('accepted!')
            self.getRoot().publish("acceptTextBox",self)
        elif name == 'backspace':
            self.setVar("text",self.getVar("text","")[:-1])
        else:
            self.setVar("text",self.getVar("text","") + chr(args['key']))
