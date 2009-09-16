# -*- coding: utf-8 -*-
from SceneNode import SceneNode
from Label import Label

class Box(SceneNode):
    def __init__(self, **args):
        SceneNode.__init__(self, **args)
        
        def intSize():
            x,y = self.getVar("size",(0,0))
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
                
        self.setVar("intSize",intSize)
    
        self.setVar("box",lambda: (0,0) + self.getVar("intSize"))
        
        #Label(parent = self, vars = dict(fg = lambda: self.getVar("fg","white"),
                                         #bg = lambda: self.getVar("bg","black"),
                                         #pos = lambda: (self.getVar("size",(0,0))[0]/2, 0),
                                         #text = lambda: self.getVar("title",""),
                                         #layer = lambda: self.getVar("layer",0) + 1,
                                         #align = "center"))
        
    def msg_textRender(self, context):
        x,y = self.getVar("globalPos")
        fg = self.getVar("fg","white")
        bg = self.getVar("bg","black")
        w,h = self.getVar("intSize",(0,0))
        corner = self.getVar("corner","+")
        hline = self.getVar("hline","-")
        vline = self.getVar("vline","|")
        fill = self.getVar("fill"," ")
        layer = self.getVar("layer",0)
        
        context.add(x,y,layer,corner,fg,bg)
        context.add(x+w-1,y,layer,corner,fg,bg)
        context.add(x,y+h-1,layer,corner,fg,bg)
        context.add(x+w-1,y+h-1,layer,corner,fg,bg)
        
        for dx in range(1,w-1):
            context.add(x+dx,y,layer,hline,fg,bg)
            context.add(x+dx,y+h-1,layer,hline,fg,bg)
        
        for dy in range(1,h-1):
            context.add(x,y+dy,layer,vline,fg,bg)
            context.add(x+w-1,y+dy,layer,vline,fg,bg)
            for dx in range(1,w-1):
                context.add(x+dx,y+dy,layer,fill,fg,bg)
