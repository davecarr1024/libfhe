from Core.Node import Node
import time

class App(Node):
    def msg_shutdown(self):
        self.shutdown = True
    
    def run(self, runTime = None):
        startTime = currentTime = lastTime = time.time()
        numFrames = 0
        self.shutdown = False
        try:
            while not self.shutdown and not runTime or currentTime - startTime < runTime:
                numFrames += 1
                currentTime = time.time()
                dtime = currentTime - lastTime
                lastTime = currentTime
                
                self.setVar("time",currentTime - startTime)
                self.publish("update",currentTime - startTime, dtime)
        finally:
            self.log("fps", float(numFrames) / (currentTime - startTime))
            if not self.shutdown:
                self.publish("shutdown")
