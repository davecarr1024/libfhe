class MessageServer:
    def __init__(self):
        self.subscribers = {}

    def subscribe(self, cmd, func):
        self.subscribers.setdefault(cmd,[]).append(func)

    def publish(self, cmd, *args, **kwargs):
        if cmd in self.subscribers:
            for func in self.subscribers[cmd]:
                func(*args,**kwargs)

messageServer = MessageServer()
