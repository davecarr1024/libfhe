from Vec2 import Vec2
from Body import Body
import time

class World:
  def __init__( self, bodies = [] ):
    self.bodies = bodies
    self.collisionPropGain = 1000
    self.collisionDiffGain = 10
    self.gravity = Vec2( 0, -1 )
    
  def run( self, minDt = 0.01 ):
    last = time.time()
    while True:
      while minDt > 0 and time.time() - last < minDt: 
        time.sleep( minDt / 10. )
      dt = time.time() - last
      last = time.time()
      
      for i in range( len( self.bodies ) ):
        for j in range( len( self.bodies ) ):
          if i != j:
            for pm in self.bodies[i].pointMasses:
              for shape in self.bodies[j].shapes:
                testResult = shape.test( pm.pos )
                if testResult != None:
                  diff, reactionPms = testResult
                  propForce = diff * self.collisionPropGain 
                  projVel = pm.vel.project( diff )
                  diffForce = projVel * -self.collisionDiffGain
                  force = propForce + diffForce
                  pm.applyForce( force )
                  
                  reactionForce = force / -float( len( reactionPms ) )
                  for reactionPm in reactionPms:
                    reactionPm.applyForce( reactionForce )
                    
              pm.applyForce( self.gravity * pm.mass )
      
      for body in self.bodies:
        body.update( dt )
        
      for body in self.bodies:
        print body.getBoundingBox(), sum( [ pm.lastForce for pm in body.pointMasses ], Vec2() )
      print '\n\n'
      
