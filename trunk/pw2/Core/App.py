# -*- coding: utf-8 -*-
from Core.Aspect import Aspect
import time

class App(Aspect):
    def msg_shutdown(self):
        self.log("shutdown")
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
                
                self.entity.setVar("time",currentTime - startTime)
                self.entity.publish("update",currentTime - startTime, dtime)
        finally:
            self.log("fps", float(numFrames) / (currentTime - startTime))
            if not self.shutdown:
                self.entity.publish("shutdown")
