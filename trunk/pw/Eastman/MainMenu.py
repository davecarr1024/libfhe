# -*- coding: utf-8 -*-
from Core.Node import Node

from Text.Label import Label
from Text.Button import Button

class MainMenu(Node):
    def onAttach(self):
        Label(parent = self, vars = dict(text = "Eastman", align = "center", pos = (0.5,0.4)))
        
        self.newGameButton = Button(parent = self, vars = dict(text = "New Game", align = "center", pos = (0.5,0.6)))
        
        self.quitButton = Button(parent = self, vars = dict(text = "Quit", align = "center", pos = (0.5,0.7)))
    
    def msg_acceptButton(self, button):
        if button == self.quitButton:
            self.parent.transition("quit")
        elif button == self.newGameButton:
            self.parent.transition("map")
