import time

class World:
  def __init__( self, bodies = [] ):
    self.bodies = bodies
    
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
                if shape.isPointInside( pm.pos ):
                  
      
      for body in self.bodies:
        body.update( dt )
