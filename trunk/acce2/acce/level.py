from player import Player
from inputMap import InputMap
from core.object import Object
from block import Block
from core.bb import BB
from core.vec2 import Vec2

import random

class Level(Object):
    def onAttach(self):
        self.defaultVar('numPlayers',1)
        self.defaultVar('playerVars',{})
        self.defaultVar('allPlayerVars',dict(material = dict(texture = 'test.jpg',
                                                             color = (1,0,0)),
                                             inputMap = dict(key_left = 'moveLeft',
                                                             key_right = 'moveRight',
                                                             key_up = 'jump',
                                                             key_lshift = 'goFast')))

        self.players = []

    def start(self):
        for i in range(1,self.numPlayers+1):
            playerVars = dict(playerNum = i)
            playerVars.update(self.allPlayerVars)
            playerVars.update(self.playerVars.get(i,{}))
            inputMapVars = playerVars.pop('inputMap',{})
            inputMap = InputMap(parent = self,
                                name = 'player_%d' % i,
                                vars = inputMapVars)
            player = Player(parent = inputMap,
                            vars = playerVars)
            self.players.append(player)
            self.spawnPlayer(player)

        self.localPublish("levelStarted")

    def spawnPlayer(self, player):
        player.setPosition(self.randomPos())
        player.setVelocity(Vec2(0,0))

    def playerDead(self, player):
        player.alive = False
        player.setPosition(Vec2(1e5,1e5))

    def getAlivePlayers(self):
        return filter(lambda obj: obj.alive, self.players)

    def getBlocks(self):
        return self.filterChildren(lambda obj: isinstance(obj,Block))

    def getBB(self):
        bbs = [block.getWorldBB() for block in self.getBlocks()]
        if not bbs:
            return BB()
        else:
            bb = bbs[0]
            map(bb.expand,bbs[1:])
            return bb

    def collides(self, v, tol = None):
        if isinstance(v,Vec2):
            if not tol:
                return any([block.getWorldBB().contains(v) for block in self.getBlocks()])
            else:
                d = Vec2(tol,tol)
                bb = BB(v+d,v-d)
                return any([block.getWorldBB().overlaps(bb) for block in self.getBlocks()])
        elif isinstance(v,BB):
            return any([block.getWorldBB().overlaps(v) for block in self.getBlocks()])
        else:
            raise TypeError

    def randomPos(self, tol = None):
        bb = self.getBB()
        size = bb.getSize()
        if size.length() < 0.1:
            self.log("return min")
            return bb.min
        if tol:
            if size.x / 2.0 < tol or size.y / 2.0 < tol:
                tol = None
        while True:
            x = random.uniform(bb.min.x,bb.max.x)
            y = random.uniform(bb.min.y,bb.max.y)
            v = Vec2(x,y)
            if not self.collides(v,tol):
                return v
