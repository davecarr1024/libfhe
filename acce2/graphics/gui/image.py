from graphics.prims2d.rect import Rect
from core.fileServer import fileServer

class Image(Rect):
    def onAttach(self):
        self.defaultVar("filename",None)
        
        self.material = dict(texture = self.filename, color = (1,1,1))
        
        self.log("attach",self.__dict__)
        
        Rect.onAttach(self)
