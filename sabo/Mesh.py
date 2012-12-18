from Shape import Shape

class Mesh( Shape ):
  def __init__( self, body, tris = [] ):
    self.body = body
    self.tris = tris
    #TODO tesselate and generate tris if not provided
    
  @staticmethod
  def distToLine( a, b, p ):
    return ( p - a ).cross( b - a ) / ( b - a ).length()
    
  @staticmethod
  def edgesEqual( edge1, edge2 ):
    return ( edge1[0] == edge2[0] and edge1[1] == edge2[1] ) or ( edge1[0] == edge2[1] and edge1[1] == edge2[0] )

  def getEdges( self ):
    edges = []
    for tri in self.tris:
      for i in range( len( tri ) ):
        edges.append( ( tri[i], tri[ ( i + 1 ) % len( tri ) ] ) )
    return edges
    
  def getPerimeter( self ):
    edges = self.getEdges()
    perimeter = []
    for i in range( len( edges ) ):
      if not any( [ i != j and self.edgesEqual( edges[i], edges[j] ) for j in range( len( edges ) ) ] ):
        perimeter.append( edges[i] )
    return perimeter
    
  def getContours( self ):
    perimeter = self.getPerimeter()
    contours = []
    while perimeter:
      contour = [ perimeter.pop() ]
      while contour[0][0] != contour[-1][1]:
        nextEdges = filter( lambda edge: contour[-1][1] in edge, perimeter )
        assert len( nextEdges ) == 1
        perimeter.remove( nextEdges[0] )
        if nextEdges[0][0] == contour[-1][1]:
          contour.append( nextEdges[0] )
        else:
          contour.append( tuple( reversed( nextEdges[0] ) ) )
      contours.append( contour )
    return contours
    
  def test( self, p ):
    inside = False
    allPoss = [ self.body.pointMasses[ i ].pos for i in range( len( self.body.pointMasses ) ) ]
    for tri in self.tris:
      poss = [ allPoss[ i ] for i in tri ]
      dists = [ self.distToLine( poss[i], poss[ ( i + 1 ) % len( poss ) ], p ) for i in range( len( poss ) ) ]
      prods = [ dists[ i ] * dists[ ( i + 1 ) % len( dists ) ] for i in range( len( dists ) ) ]
      if all( [ prod > 0 for prod in prods ] ):
        inside = True
        break
    if inside:
      perimeter = self.getPerimeter()
      dists = [ self.distToLine( allPoss[ edge[0] ], allPoss[ edge[1] ], p ) for edge in perimeter ]
      minDistIndex = min( range( len( dists ) ), key = lambda i: abs( dists[i] ) )
      minDist = dists[ minDistIndex ]
      minEdge = perimeter[ minDistIndex ]
      minNorm = ( allPoss[ minEdge[1] ] - allPoss[ minEdge[0] ] ).norm().perp()
      diff = minNorm * minDist
      return diff

if __name__ == '__main__':
  from PointMass import PointMass
  from Body import Body
  from Vec2 import Vec2

  #ccw tri
  pms = [ PointMass( Vec2( 0, 0 ) ), PointMass( Vec2( 1, 0 ) ), PointMass( Vec2( 0, 1 ) ) ]
  body = Body( pms, [] )
  mesh = Mesh( body, [ ( 0, 1, 2 ) ] )
  assert not mesh.test( Vec2( 0.6, 0.6 ) )
  assert not mesh.test( Vec2( 0.25, -0.1 ) )
  assert not mesh.test( Vec2( -0.1, 0.25 ) )
  assert Vec2( 0.1, 0.1 ) == mesh.test( Vec2( 0.4, 0.4 ) )
  assert Vec2( -0.1, 0 ) == mesh.test( Vec2( 0.1, 0.2 ) )
  assert Vec2( 0, -0.1 ) == mesh.test( Vec2( 0.2, 0.1 ) )
  
  #cw tri
  pms = [ PointMass( Vec2( 0, 0 ) ), PointMass( Vec2( 1, 0 ) ), PointMass( Vec2( 0, 1 ) ) ]
  body = Body( pms, [] )
  mesh = Mesh( body, [ ( 2, 1, 0 ) ] )
  assert not mesh.test( Vec2( 0.6, 0.6 ) )
  assert not mesh.test( Vec2( 0.25, -0.1 ) )
  assert not mesh.test( Vec2( -0.1, 0.25 ) )
  assert Vec2( 0.1, 0.1 ) == mesh.test( Vec2( 0.4, 0.4 ) )
  assert Vec2( -0.1, 0 ) == mesh.test( Vec2( 0.1, 0.2 ) )
  assert Vec2( 0, -0.1 ) == mesh.test( Vec2( 0.2, 0.1 ) )
  
  #square
  pms = [ PointMass( Vec2( 0, 0 ) ), 
          PointMass( Vec2( 1, 0 ) ), 
          PointMass( Vec2( 0, 1 ) ),
          PointMass( Vec2( 1, 1 ) ) ]
  body = Body( pms, [] )
  mesh = Mesh( body, [ ( 0, 1, 2 ), ( 1, 2, 3 ) ] )
  assert not mesh.test( Vec2( 0.5, -0.1 ) )
  assert not mesh.test( Vec2( 0.5, 1.1 ) )
  assert not mesh.test( Vec2( -0.1, 0.5 ) )
  assert not mesh.test( Vec2( 1.1, 0.5 ) )
  assert Vec2( -0.1, 0 ) == mesh.test( Vec2( 0.1, 0.5 ) )
  assert Vec2( 0.4, 0 ) == mesh.test( Vec2( 0.6, 0.5 ) )
  
  assert len( mesh.getContours() ) == 1
  assert len( mesh.getContours()[0] ) == 4
