from core.object import Object
from core.vec2 import Vec2
from core.fileServer import fileServer

from graphics.prims2d.text import Text
from graphics.gui.spinner import Spinner
from graphics.gui.button import Button

class OpeningMenu(Object):
    def onAttach(self):
        Text(parent = self,
             vars = dict(position = Vec2(0.5,0.25),
                         text = "ACCE",
                         material = dict(color = (0,0,0))))

        levels = fileServer.enumerateFiles("acce/data","*.lvl")

        self.spinner = Spinner(parent = self,
                               vars = dict(position = Vec2(0.25,0.5),
                                           scale = Vec2(0.5,0.2),
                                           options = levels))

        self.loadButton = Button(parent = self,
                                 vars = dict(position = Vec2(0.4,0.7),
                                             scale = Vec2(0.2,0.1),
                                             text = "Load"))

    def msg_buttonClicked(self, args):
        if self.loadButton == args.get('sender'):
            level = self.spinner.getSelection()
            self.publish("loadLevel", level = level)
            self.publish("popMenu")
