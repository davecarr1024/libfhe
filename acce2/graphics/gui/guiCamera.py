from graphics.prims2d.sceneNode import SceneNode
from core.optionServer import optionServer
from core.vec2 import Vec2

class GuiCamera(SceneNode):
    def onAttach(self):
        self.defaultVar("static",False)
        scale = Vec2(*optionServer.setdefault('res',(800,600)))
        self.defaultVar("scale",scale)
        SceneNode.onAttach(self)
