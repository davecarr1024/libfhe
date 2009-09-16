from guiCamera import GuiCamera
from core import util

class MenuServer(GuiCamera):
    def onAttach(self):
        GuiCamera.onAttach(self)
        self.menu = None
        self.menuStack = []
    
    def msg_pushMenu(self, args):
        assert 'type' in args
        self.push(util.dynload(args['type'])(vars = args))

    def msg_popMenu(self, args):
        self.pop(**args)

    def push(self, menu):
        if self.menu:
            self.removeChild(self.menu)
            self.menu.detachFromParent()
            self.menuStack.append(self.menu)
        self.menu = menu
        self.menu.attachToParent(self)

    def pop(self, **args):
        if self.menu:
            self.publish("poppedMenu",menu = self.menu, **args)
            self.menu.delete()
        if self.menuStack:
            self.menu = self.menuStack.pop()
            self.menu.attachToParent(self)
        else:
            self.menu = None
