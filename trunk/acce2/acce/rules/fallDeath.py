from core.object import Object
from acce.level import Level

class FallDeath(Object):
    def onAttach(self):
        self.defaultVar("decLives",True)
        self.defaultVar("miny",-20)

        self.level = self.searchAncestors(lambda obj: isinstance(obj,Level))
        assert self.level

    def msg_tick(self, args):
        for player in self.level.players:
            if player.alive and player.position.y < self.miny:
                if self.decLives:
                    player.lives -= 1
                    if player.lives >= 0:
                        self.level.spawnPlayer(player)
                    else:
                        self.level.playerDead(player)
                else:
                    self.level.spawnPlayer(player)
