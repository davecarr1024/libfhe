from core.object import Object

class InputMap(Object):
    def dispatch(self, cmd, f, b):
        if hasattr(self,cmd):
            self.localPublish(getattr(self,cmd),f = f, b = b)

    def msg_keyDown(self, args):
        self.dispatch("key_%s" % args['name'],1,True)

    def msg_keyUp(self, args):
        self.dispatch("key_%s" % args['name'],0,False)

    def msg_joyButtonDown(self, args):
        self.dispatch("joyButton_%d_%d" % (args['joy'],args['button']),1,True)

    def msg_joyButtonUp(self, args):
        self.dispatch("joyButton_%d_%d" % (args['joy'],args['button']),0,False)

    def msg_joyAxisMotion(self, args):
        f = args['value']
        if f < 0:
            f *= -1
            suffix = 'pos'
        else:
            suffix = 'neg'
        b = f > 0.5
        self.dispatch("joyAxis_%d_%d_%s" % (args['joy'],args['axis'],suffix),f,b)

    def msg_mouseButtonDown(self, args):
        self.dispatch("mouseButton_%s" % args['buttonName'],1,True)

    def msg_mouseButtonUp(self, args):
        self.dispatch("mouseButton_%s" % args['buttonName'],0,False)
