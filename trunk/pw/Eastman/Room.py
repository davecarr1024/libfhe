from Core.Node import Node
from RoomPlayer import RoomPlayer
from Text.SceneNode import SceneNode
from Text.Box import Box

class Room(Node):
    def onAttach(self):
        self.player = RoomPlayer(parent = self, vars = dict(pos = self.getVar("playerPos",(0,0))))
        self.lastPlayerPos = self.player.getVar("pos",(0,0))
        
    def getGeom(self):
        return self.filterDescendents(lambda node: not node.hasAncestor(self.player) and isinstance(node,SceneNode), True)
        
    def msg_update(self, time, dtime):
        playerPos = self.player.getVar("pos",(0,0))
        if playerPos != self.lastPlayerPos:
            for geom in self.getGeom():
                if geom.overlaps(self.player):
                    self.player.setVar("pos",self.lastPlayerPos)
                    if geom.hasVar("contactRoom") and geom.hasVar("contactPos"):
                        self.parent.enterRoom(geom.getVar("contactRoom"),geom.getVar("contactPos"))
            self.lastPlayerPos = playerPos
