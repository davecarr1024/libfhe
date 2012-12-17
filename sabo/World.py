import time

class Sabo:
  def __init__( self, bodies = [] ):
    self.bodies = bodies
    
  def run( self, minDt = 0.01 ):
    last = time.time()
    while True:
      while minDt > 0 and time.time() - last < minDt: 
        time.sleep( minDt / 10. )
      dt = time.time() - last
      last = time.time()
      for body in self.bodies:
        body.update( dt )
