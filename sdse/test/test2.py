print "hello from script 2", self.getName()

self.getEntity("scriptEnt").setVar("testVar","hello from this other script via a var")

self.publish("testMsg","hello from this other script via a message",2,3)
