from util import feq
import math

class Vec2:
  def __init__(self, x, y):
    self.x = float(x)
    self.y = float(y)
    
  def __repr__(self):
    return 'Vec2(%f, %f)' % (self.x, self.y)
    
  def __eq__(self, v):
    return feq(self.x, v.x) and feq(self.y, v.y)
    
  def __getitem__(self, i):
    if i == 0:
      return self.x
    elif i == 1:
      return self.y
    else:
      raise KeyError(i)
      
  def __setitem__(self, i, v):
    if i == 0:
      self.x = v
    elif i == 1:
      self.y = v
    else:
      raise KeyError(i)
    
  def __neg__(self):
    return Vec2(-self.x, -self.y)
    
  def __add__(self, v):
    return Vec2(self.x + v.x, self.y + v.y)
    
  def __sub__(self, v):
    return Vec2(self.x - v.x, self.y - v.y)
    
  def __mul__(self, v):
    if isinstance(v, Vec2):
      return Vec2(self.x * v.x, self.y * v.y)
    else:
      return Vec2(self.x * v, self.y * v)
      
  def __div__(self, v):
    if isinstance(v, Vec2):
      return Vec2(self.x / v.x, self.y / v.y)
    else:
      return Vec2(self.x / v, self.y / v)
      
  def sq_length(self):
    return self.x * self.x + self.y * self.y
    
  def length(self):
    return math.sqrt(self.sq_length())
    
  def angle(self):
    return math.atan2(self.y, self.x)
    
  def norm(self):
    return self / self.length()
    
  def dot(self, v):
    return self.x * v.x + self.y * v.y
    
  def lerp(self, v, i):
    return self + ( v - self ) * i
    
  def rotate(self, r):
    a = self.angle()
    l = self.length()
    return Vec2(l * math.cos(r + a), l * math.sin(r + a))
    
  @staticmethod
  def zero():
    return Vec3(0, 0, 0)
    
  @staticmethod
  def test():
    assert Vec2(1, 2) + Vec2(3, 4) == Vec2(4, 6)
    assert Vec2(1, 2) - Vec2(3, 4) == Vec2(-2, -2)
    assert Vec2(1, 2) * Vec2(3, 4) == Vec2(3, 8)
    assert Vec2(1, 2) * 3 == Vec2(3, 6)
    assert Vec2(10, 15) / Vec2(2, 5) == Vec2(5, 3)
    assert Vec2(10, 15) / 5 == Vec2(2, 3)
    assert feq(Vec2(3, 4).sq_length(), 25)
    assert feq(Vec2(3, 4).length(), 5)
    assert feq(Vec2(0, 1).angle(), math.pi/2)
    assert Vec2(0, -10).norm() == Vec2(0, -1)
    assert feq(Vec2(1, 2).dot(Vec2(3, 4)), 11)
    assert Vec2(0, 0).lerp(Vec2(10, 20), .5) == Vec2(5, 10)
    assert Vec2(1, 0).rotate(math.pi * 1.5) == Vec2(0, -1)
    
Vec2.test()

