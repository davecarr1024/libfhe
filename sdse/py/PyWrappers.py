def setupWrappers(pyEntity):
    def publish(self, cmd, *args):
        self._publish(cmd,list(args))
    pyEntity.publish = publish
