from Mesh import Mesh
from PointMass import PointMass
from Spring import Spring
from Vec2 import Vec2

class Body:
  def __init__( self, pointMasses = [], springs = [], shapes = [], pin = False ):
    self.pointMasses = pointMasses
    self.springs = springs
    self.shapes = shapes
    
    for shape in self.shapes:
      shape.body = self
    
    if self.pointMasses and not self.springs:
      for i in range( len( self.pointMasses ) ):
        if pin:
          self.springs.append( Spring( self.pointMasses[i] ) )
        else:
          for j in range( i + 1, len( self.pointMasses ) ):
            self.springs.append( Spring( self.pointMasses[i], self.pointMasses[j] ) )
    
  @staticmethod
  def rectangle( pos, size, pin = False ):
    pms = []
    for dx in ( -0.5, 0.5 ):
      for dy in ( -0.5, 0.5 ):
        pms.append( PointMass( pos + size * Vec2( dx, dy ) ) )
    return Body( pms, [], [ Mesh( [ ( 0, 1, 2 ), ( 1, 2, 3 ) ] ) ], pin )
    
  def update( self, dt ):
    for spring in self.springs:
      spring.update( dt )
  
    for pointMass in self.pointMasses:
      pointMass.update( dt )

  def getCenter( self ):
    return sum( [ pm.pos for pm in self.pointMasses ], Vec2() ) / float( len( self.pointMasses ) )
    
  def getRadius( self ):
    center = self.getCenter()
    return max( [ ( pm.pos - center ).length() for pm in self.pointMasses ] )
    
  def getBoundingBox( self ):
    return ( Vec2( min( [ pm.pos.x for pm in self.pointMasses ] ),
                   min( [ pm.pos.y for pm in self.pointMasses ] ) ),
             Vec2( max( [ pm.pos.x for pm in self.pointMasses ] ),
                   max( [ pm.pos.y for pm in self.pointMasses ] ) ) )
                   
  def getBoundingBoxSize( self ):
    min, max = self.getBoundingBox()
    return max - min
