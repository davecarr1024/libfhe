from Shape import Shape

class Circle( Shape ):
  def __init__( self, pointMasses = [] ):
    self.pointMasses = pointMasses
    
  def isPointInside( self, p ):
    center = sum( [ pm.pos for pm in self.pointMasses ], Vec2() ) / float( len( self.pointMasses ) )
    squaredRadius = max( [ ( pm.pos - center ).squaredLength() for pm in self.pointMasses ] )
    return ( p - center ).squaredLength() <= squaredRadius
