from Vec2 import Vec2

class PointMass:
  def __init__( self, pos, mass = 1 ):
    self.pos = pos
    self.vel = Vec2()
    self.force = Vec2()
    self.mass = float( mass )
    self.friction = 0.1
    self.lastForce = Vec2()
    
  def applyForce( self, f ):
    self.force += f
    
  def update( self, dt ):
    acc = self.force / self.mass
    self.vel += ( acc - self.vel * self.friction ) * dt
    self.pos += self.vel * dt
    self.lastForce = self.force
    self.force = Vec2()
