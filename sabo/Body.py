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
        print pos, size, dx, dy, pms[-1].pos
    return Body( pms, [], [ Mesh( [ ( 0, 1, 2 ), ( 1, 2, 3 ) ] ) ], pin )
    
  def update( self, dt ):
    for spring in self.springs:
      spring.update( dt )
  
    for pointMass in self.pointMasses:
      pointMass.update( dt )

  def getCenterOfMass( self ):
    return sum( [ pm.pos for pm in self.pointMasses ], Vec2() ) / float( len( self.pointMasses ) )
      