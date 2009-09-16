# -*- coding: utf-8 -*-
from Core.Node import Node
from Text.Label import Label

class Splash(Node):
    def onAttach(self):
        label = Label(parent = self,
                      vars = dict(text = "Flyin' Hawaiian Productions Proudly Presents",
                                  align = "center",
                                  pos = (0.5,0.5)))
        
    def msg_update(self, time, dtime):
        if time - self.defaultVar("startTime",time) > self.getVar("hangTime",5):
            self.parent.transition("mainMenu")
            
    def msg_textInput(self, ch):
        self.parent.transition("mainMenu")
