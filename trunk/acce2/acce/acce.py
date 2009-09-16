from core.object import Object
from graphics.gui.menuServer import MenuServer
from camera import Camera
from core.vec3 import Vec3
from level import Level

class Acce(Object):
    def onAttach(self):
        self.camera = Camera(parent = self,
                             name = "camera")
        self.publish("pushMenu",type = "acce.menus.openingMenu")
        self.level = None

    def msg_loadLevel(self, args):
        assert 'level' in args
        self.level = Level(parent = self.camera,
                           vars = args.get('vars',{}),
                           children = args.get('children',{}))
        self.level.load(args['level'])
        self.level.start()
        self.camera.attachToLevel(self.level)

    def msg_unloadLevel(self, args):
        self.camera.detachFromLevel()
        self.level = None
