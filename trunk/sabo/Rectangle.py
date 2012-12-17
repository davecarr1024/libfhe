from Shape import Shape

class Rectangle( Shape ):
  def __init__( self, pointMasses = [] ):
    self.pointMasses = pointMasses
    
  def isPointInside( self, p ):
    minx = min( [ pm.pos.x for pm in self.pointMasses ] )
    maxx = max( [ pm.pos.x for pm in self.pointMasses ] )
    miny = min( [ pm.pos.y for pm in self.pointMasses ] )
    maxy = max( [ pm.pos.y for pm in self.pointMasses ] )
    dists = ( p.x - minx, maxx - p.x, p.y - miny, maxy - p.y )
    if all( [ dist >= 0 for dist in dists ] ):
      minDistIndex = min( range( len( dists ) ), key = lambda i: dists[i] )
      if minDistIndex == 0:
        return Vec2( -dists[0], 0 )
      elif minDistIndex == 1:
        return Vec2( dists[1], 0 )
      elif minDistIndex == 2:
        return Vec2( 0, -dists[2] )
      elif minDistIndex == 3:
        return Vec2( 0, dists[3] )
