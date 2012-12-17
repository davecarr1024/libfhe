from Shape import Shape

class Polygon( Shape ):
  def __init__( self, pointMasses = [] ):
    self.pointMasses = pointMasses
    
  @staticmethod
  def distToLine( a, b, p ):
    return ( p - a ).cross( b - a ) / ( b - a ).length()
    
  def isPointInside( self, p ):
    if len( self.pointMasses ) >= 3:
      poss = [ pm.pos for pm in self.pointMasses ]
      dists = [ self.distToLine( poss[i], poss[ ( i + 1 ) % len( poss ) ], p ) for i in range( len( poss ) ) ]
      if all( lambda i: dists[i] * dists[ ( i + 1 ) % len( dists ) ] > 0, range( len( dists ) ) ):
        minDistIndex = min( range( len( dists ) ), key = lambda i: dists[i] )
        minDist = dists[ minDistIndex ]
        norm = ( poss[ ( minDistIndex + 1 ) % len( poss ) ] - poss[ minDistIndex ]  ).perp()
        return norm * -minDist
