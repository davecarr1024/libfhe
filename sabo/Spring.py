from Vec2 import Vec2

class Spring:
  def __init__( self, p1, p2 = None ):
    self.p1 = p1
    self.p2 = p2
    self.propGain = 100
    self.diffGain = 10
    if self.p1 and self.p2:
      self.baseDist = ( self.p1.pos - self.p2.pos ).length()
    elif self.p1:
      self.basePos = Vec2( self.p1.pos.x, self.p1.pos.y )
    else:
      raise RuntimeError( 'no pointMasses provided' )
    
  def update( self, dt ):
    if self.p1 and self.p2:
      diff = self.p2.pos - self.p1.pos
      diffLength = diff.length()
      normDiff = diff.norm()
      
      propForce = normDiff * ( ( diffLength - self.baseDist ) * self.propGain )
      
      projVel1 = self.p1.vel.project( normDiff )
      projVel2 = self.p2.vel.project( normDiff )
      velDiff = projVel2 - projVel1
      diffForce = velDiff * self.diffGain
      
      force = propForce + diffForce
      
      self.p1.applyForce( force )
      self.p2.applyForce( -force )
      
    elif self.p1:
      diff = self.basePos - self.p1.pos
      if diff.length():
        propForce = diff * self.propGain
        diffForce = self.p1.vel.project( diff ) * self.diffGain
        force = propForce - diffForce
        self.p1.applyForce( force )
      
    else:
      raise RuntimeError( 'no pointMasses provided' ) 
