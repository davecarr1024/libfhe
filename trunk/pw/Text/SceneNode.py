# -*- coding: utf-8 -*-
from Core.Node import Node

class SceneNode(Node):
    def __init__(self, **args):        
        Node.__init__(self, **args)
        
        def intPos():
            x,y = self.getVar("pos",(0,0))
            parent = self.getParentSceneNode()
            if parent:
                pminx, pminy, pmaxx, pmaxy = parent.getVar("box",(0,0,0,0))
                psizex = pmaxx - pminx
                psizey = pmaxy - pminy
                if not psizex or not psizey:
                    return 0,0
                if isinstance(x,int) and isinstance(y,int):
                    return x % psizex, y % psizey
                else:
                    return int(float(x) * psizex) % psizex, int(float(y) * psizey) % psizey
            else:
                return 0,0
                
        self.setVar("intPos",intPos)
    
        def globalPos():
            parent = self.getParentSceneNode()
            if parent:
                px, py = parent.getVar("globalPos",(0,0))
                x,y = self.getVar("intPos",(0,0))
                return px + x, py + y
            else:
                return self.getVar("intPos",(0,0))
        
        self.setVar("globalPos",globalPos)
                
        def globalBox():
            minx,miny,maxx,maxy = self.getVar("box",(0,0,0,0))
            for child in self.getChildSceneNodes():
                cx, cy = child.getVar("pos",(0,0))
                cminx, cminy, cmaxx, cmaxy = child.getVar("globalBox")
                minx = min(minx,cminx+cx)
                miny = min(miny,cminy+cy)
                maxx = max(maxx,cmaxx+cx)
                maxy = max(maxy,cmaxy+cy)
            return minx,miny,maxx,maxy
        
        self.setVar("globalBox",globalBox)
        
    def getParentSceneNode(self):
        return self.searchAncestors(lambda node: isinstance(node,SceneNode))
    
    def getChildSceneNodes(self):
        return self.filterDescendents(lambda node: isinstance(node,SceneNode), False)
    
    def getWindow(self):
        from Window import Window
        return self.searchAncestors(lambda node: isinstance(node,Window))
        
    def getRelPos(self, node):
        gx, gy = self.getVar("globalPos")
        ngx, ngy = node.getVar("globalPos")
        
        return ngx - gx, ngy - gy
    
    def overlaps(self, node):
        nx, ny = self.getRelPos(node)
        nminxr, nminyr, nmaxxr, nmaxyr = node.getVar("globalBox")
        nminx, nminy, nmaxx, nmaxy = nminxr + nx, nminyr + ny, nmaxxr + nx, nmaxyr + ny
        minx, miny, maxx, maxy = self.getVar("globalBox")
        
        self.log("overlaps",self,node)
        self.log("node pos", nx, ny, "box", nminxr, nminyr, nmaxxr, nmaxyr, "moved box", nminx,nminy,nmaxx,nmaxy)
        self.log("my box", minx,miny,maxx,maxy)
        
        t = minx < nmaxx and maxx > nminx and miny < nmaxy and maxy > nminy
        
        self.log("overlaps?",t)
        return t

    @staticmethod
    def test():
        n1 = SceneNode(vars = dict(pos = (10,10)))
        n2 = SceneNode(parent = n1, vars = dict(pos = (-5,-15), box = (-10,-10,10,10)))
        assert n2.getVar("globalPos") == (5,-5)
        assert n2.getVar("globalBox") == (-10,-10,10,10)
        assert n1.getVar("globalBox") == (-15,-25,5,-5)
        
        n3 = SceneNode(parent = n1, vars = dict(pos = (-100,-100), box = (-10,-10,10,10)))
        assert n1.getVar("globalBox") == (-110,-110,5,-5)
        assert not n2.overlaps(n3)
        
        n3.setVar("pos",(-10,-20))
        assert n2.overlaps(n3)
