from Text.SceneNode import SceneNode
from Text.Box import Box
from Text.Label import Label
from Text.Button import Button

from Room import Room
from RoomPlayer import RoomPlayer

class NPC(SceneNode):
    def onAttach(self):
        self.head = Label(parent = self, 
                          vars = dict(text = "o",
                                      fg = lambda: self.getVar("fg","white"),
                                      bg = lambda: self.getVar("bg","black")))
        self.legs = Label(parent = self,
                          vars = dict(text = "X",
                                      fg = lambda: self.getVar("fg","white"),
                                      bg = lambda: self.getVar("bg","black"),
                                      pos = (0,1)))
                                      
    def getRoom(self):
        room = self.searchAncestors(lambda node: isinstance(node,Room))
        assert room
        return room
        
    def getPlayer(self):
        player = self.getRoom().searchDescendents(lambda node: isinstance(node,RoomPlayer))
        assert player
        return player
        
    def msg_textInput(self, ch):
        if ch == self.getVar("talkChar",10):
            x,y = self.getVar("globalPos",(0,0))
            px,py = self.getPlayer().getVar("globalPos",(0,0))
            if abs(px - x) <= 1 and abs(py - y) < 2:
                self.talk(None)
                
    def talk(self, branch):
        if not branch:
            branch = self.getVar("conv")
            assert branch
            
        self.box = Label
