class Body:
  def __init__( self, pointMasses = [], springs = [], shapes = [] ):
    self.pointMasses = pointMasses
    self.springs = springs
    self.shapes = shapes
    
  def update( self, dt ):
    for spring in self.springs:
      spring.update( dt )
  
    for pointMass in self.pointMasses:
      pointMass.update( dt )
