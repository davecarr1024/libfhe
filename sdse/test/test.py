print "hello from python", self, dir(self)

self.setVar('testBool',False)
self.setVar('testInt',31)
self.setVar('testFloat',5.124)
self.setVar('testString','hello')

print self.getVar('testBool'), self.getVar('testInt'), self.getVar('testFloat'), self.getVar('testString'), self.getVar('testDefault','defaultVal')

print self.addAspect('test/TestAspect','testAspect')
print self.addAspect('test/TestAspect','testAspect')

self.deleteAspect('testAspect_2')

def setVar_testVar(self, val):
    print "setVar_testVar", val, self.getVar('testVar')

def msg_testMsg(self, args):
    print "msg_testMsg", args, self.getVar('testInt','default')

self.setVar("v2",Vector2(1,2))
print self.getVar("v2")

self.setVar("v3",Vector3(10,11,12))
print self.getVar("v3")

self.setVar("q",Quaternion(45,Vector3(1,0,0)))
print self.getVar("q")

print self.getVar("fileVector2"), self.getVar("fileVector3"), self.getVar("fileQuaternion")
