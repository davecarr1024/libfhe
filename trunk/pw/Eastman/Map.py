# -*- coding: utf-8 -*-
from Core.Node import Node

class Map(Node):
    def onAttach(self):
        self.room = None
        firstRoom = self.getVar("firstRoom",None)
        firstRoomPos = self.getVar("firstRoomPos",(0,0))
        assert firstRoom
        self.enterRoom(firstRoom,firstRoomPos)
        
    def enterRoom(self, roomFile, playerPos):
        if self.room:
            self.removeChild(self.room)
        self.room = Node.load(roomFile)
        self.room.setVar("playerPos",playerPos)
        self.addChild(self.room)
