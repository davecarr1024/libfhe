from graphics.prims3d.camera import Camera as GraphicsCamera
from core.vec3 import Vec3
from core.bb import BB

class Camera(GraphicsCamera):
    def onAttach(self):
        GraphicsCamera.onAttach(self)
        self.defaultVar("desiredPosition",self.position)
        self.defaultVar("moveTime",0.1)
        self.defaultVar("minDist",50)

        self.level = None

    def attachToLevel(self, level):
        self.level = level
        self.position = self.desiredPosition = self.posForLevel(self.level,False)

    def detachFromLevel(self):
        self.level = None

    def posForLevel(self, level, followPlayers = True):
        players = self.level.getAlivePlayers()
        if players and followPlayers:
            bb = players[0].getWorldBB()
            map(bb.expand,[player.getWorldBB() for player in players[1:]])
        else:
            bb = self.level.getBB()
        center = bb.getCenter()
        size = bb.getSize()
        return Vec3(center.x,center.y,max(size.length(),self.minDist))

    def msg_tick(self, args):
        if self.level:
            self.desiredPosition = self.posForLevel(self.level)
        dtime = args['dtime']
        l = dtime / self.moveTime
        self.position = self.position.lerp(self.desiredPosition,l)
        self.lookAt = self.position + Vec3(0,0,-10)
