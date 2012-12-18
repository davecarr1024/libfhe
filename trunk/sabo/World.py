from Vec2 import Vec2
import time

class World:
  def __init__( self, bodies = [] ):
    self.bodies = bodies
    self.collisionPropGain = 10
    self.collisionDiffGain = 1
    self.gravity = Vec2( 0, -10 )
    
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
                diff = shape.test( pm.pos )
                if diff != None:
                  propForce = diff * self.collisionPropGain 
                  diffForce = pm.vel.project( diff ) * -self.collisionDiffGain
                  pm.applyForce( propForce + diffForce )
              pm.applyForce( self.gravity * pm.mass )
      
      for body in self.bodies:
        body.update( dt )
