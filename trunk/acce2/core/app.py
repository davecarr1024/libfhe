from object import Object
from time import time

class App(Object):
    def __init__(self, *args, **kwargs):
        Object.__init__(self,args,kwargs)
        self.shutdown = False
        
    def msg_shutdownApp(self, args):
        self.shutdown = True

    def run(self, maxTime = None):
        lastTime = startTime = time()
        frameCount = 0
        
        while not self.shutdown and (not maxTime or lastTime - startTime < maxTime):
            frameCount += 1
            
            now = time()
            dtime = now - lastTime
            lastTime = now
            
            self.publish("tick",time = now - startTime, dtime = dtime)
            
        self.log("fps",float(frameCount)/(time()-startTime))
