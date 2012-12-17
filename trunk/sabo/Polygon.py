from Shape import Shape

class Polygon( Shape ):
  def __init__( self, pointMasses = [] ):
    self.pointMasses = pointMasses
    
  def isPointInside( self, p ):
    if len( self.pointMasses ) < 3:
      return False
    side = Vec2.classify( self.pointMasses[-1].pos, self.pointMasses[0].pos, p )
    for i in range( 0, len( self.pointMasses ) - 1 ):
      if Vec2.classify( self.pointMasses[i].pos, self.pointMasses[ i + 1 ].pos, p ) * side < 0:
        return False
    return True
