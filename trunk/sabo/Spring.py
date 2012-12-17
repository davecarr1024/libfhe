class Spring:
  def __init__( self, p1, p2 ):
    self.p1 = p1
    self.p2 = p2
    self.p = 50
    self.d = 10
    self.baseDist = ( p1.pos - p2.pos ).length()
    
  def update( self, dt ):
    diff = self.p2.pos - self.p1.pos
    diffLength = diff.length()
    normDiff = diff.norm()
    
    propForce = normDiff * ( ( diffLength - self.baseDist ) * self.p )
    
    projVel1 = self.p1.vel.project( normDiff )
    projVel2 = self.p2.vel.project( normDiff )
    velDiff = projVel2 - projVel1
    diffForce = velDiff * self.d
    
    force = propForce + diffForce
    
    self.p1.applyForce( force )
    self.p2.applyForce( -force )
