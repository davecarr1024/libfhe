# -*- coding: utf-8 -*-
from Text.SceneNode import SceneNode
from Text.Label import Label

class RoomPlayer(SceneNode):
    def onAttach(self):
        def legText():
            time = self.getRoot().getVar("time",0)
            if time - self.getVar("lastMoveTime",0) < self.getVar("moveAnimTime",0.25):
                return "XY"[int(self.getRoot().getVar("time",0) / self.getVar("legAnimTime",0.25))%2]
            else:
                return "X"
        
        self.head = Label(parent = self, vars = dict(text = "o"))
        self.legs = Label(parent = self, 
                          vars = dict(text = legText, 
                                      pos = (0,1)))
                                      
    def msg_textInput(self, ch):
        if ch == self.getVar("upChar",ord('w')):
            self.move(0,-1)
        elif ch == self.getVar("downChar",ord('s')):
            self.move(0,1)
        elif ch == self.getVar("leftChar",ord('a')):
            self.move(-1,0)
        elif ch == self.getVar("rightChar",ord('d')):
            self.move(1,0)
            
    def move(self, dx, dy):
        self.setVar("lastMoveTime",self.getRoot().getVar("time",0))
        x,y = self.getVar("pos",(0,0))
        self.setVar("pos",(x+dx,y+dy))
