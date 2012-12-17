from Shape import Shape

class Rectangle( Shape ):
  def __init__( self, pointMasses = [] ):
    self.pointMasses = pointMasses
    
  def isPointInside( self, p ):
    minx = min( [ pm.pos.x for pm in self.pointMasses ] )
    maxx = max( [ pm.pos.x for pm in self.pointMasses ] )
    miny = min( [ pm.pos.y for pm in self.pointMasses ] )
    maxy = max( [ pm.pos.y for pm in self.pointMasses ] )
    return p.x >= minx and p.x <= maxx and p.y >= miny and p.y <= maxy
    