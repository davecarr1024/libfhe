from core.object import Object
from acce.level import Level
from graphics.prims2d.text import Text
from core.vec2 import Vec2

class LivesDisplay(Object):
    def msg_levelStarted(self, args):
        self.level = self.searchAncestors(lambda obj: isinstance(obj,Level))
        assert self.level

        w = 1.0 / len(self.level.players)
        self.text = []
        for i, player in enumerate(self.level.players):
            pos = Vec2(w * i + w/2)
            text = Text(parent = self,
                        vars = dict(position = Vec2(w * i + w/2),
                                    scale = Vec2(w,0.2),
                                    static = False,
                                    material = dict(color = (0,0,0))))
            self.text.append(text)

    def updateLives(self):
        for text, player in zip(self.text,self.level.players):
            if player.alive:
                text.text = "%d: %d" % (player.playerNum,player.lives)
            else:
                text.text = "%d: ~" % player.playerNum

    def msg_tick(self, args):
        self.updateLives()
