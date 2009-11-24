from MessageServer import messageServer
import time

class App:
    def __init__(self):
        messageServer.subscribe('shutdown',self.shutdown)

    def shutdown(self):
        self.doShutdown = True
    
    def run(self, maxTime = -1):
        self.doShutdown = False
        startTime = time.time()
        lastTime = currentTime = 0

        while not self.doShutdown and (maxTime < 0 or currentTime < maxTime):
            currentTime = time.time() - startTime
            dtime = currentTime - lastTime
            lastTime = currentTime
            messageServer.publish("update",currentTime,dtime)

app = App()
