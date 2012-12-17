import math

class Vec2:
  def __init__( self, x = 0, y = 0 ):
    self.x = float( x )
    self.y = float( y )
    
  def __repr__( self ):
    return 'Vec2( %f, %f )' % ( self.x, self.y )
    
  def __eq__( self, v, eps = 1e-5 ):
    return abs( self.x - v.x ) <= eps and abs( self.y - v.y ) <= eps
    
  def __neg__( self ):
    return Vec2( -self.x, -self.y )
    
  def __add__( self, v ):
    return Vec2( self.x + v.x, self.y + v.y )
    
  def __sub__( self, v ):
    return Vec2( self.x - v.x, self.y - v.y )
  
  def __mul__( self, v ):
    if isinstance( v, Vec2 ):
      return Vec2( self.x * v.x, self.y * v.y )
    else:
      return Vec2( self.x * v, self.y * v )
      
  def __div__( self, v ):
    if isinstance( v, Vec2 ):
      return Vec2( self.x / v.x, self.y / v.y )
    else:
      return Vec2( self.x / v, self.y / v )
      
  def dot( self, v ):
    return self.x * v.x + self.y * v.y
    
  def squaredLength( self ):
    return self.x * self.x + self.y * self.y
    
  def length( self ):
    return math.sqrt( self.x * self.x + self.y * self.y )
    
  def norm( self ):
    return self / self.length()
    
  def project( self, v ):
    return v * ( self.dot( v ) / v.squaredLength() )
    
  def cross( self, v ):
    return self.x * v.y - self.y * v.x

  #which side of the line formed by a and b is c on?
  @staticmethod
  def classify( a, b, c ):
    return ( b - a ).cross( c - a )
    #return ( b.x - a.x ) * ( c.y - a.y ) - ( b.y - a.y ) * ( c.x - a.x )
    