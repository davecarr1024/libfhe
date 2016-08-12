from util import feq
import math

class Vec3:
  def __init__(self, x, y, z):
    self.x = float(x)
    self.y = float(y)
    self.z = float(z)
    
  def __repr__(self):
    return 'Vec3(%f, %f, %f)' % (self.x, self.y, self.z)
    
  def __eq__(self, v):
    return feq(self.x, v.x) and feq(self.y, v.y) and feq(self.z, v.z)
    
  def __getitem__(self, i):
    if i == 0:
      return self.x
    elif i == 1:
      return self.y
    elif i == 2:
      return self.z
    else:
      raise KeyError(i)
      
  def __setitem__(self, i, v):
    if i == 0:
      self.x = v
    elif i == 1:
      self.y = v
    elif i == 2:
      self.z = v
    else:
      raise KeyError(i)
    
  def __neg__(self):
    return Vec3(-self.x, -self.y, -self.z)
    
  def __add__(self, v):
    return Vec3(self.x + v.x, self.y + v.y, self.z + v.z)
    
  def __sub__(self, v):
    return Vec3(self.x - v.x, self.y - v.y, self.z - v.z)
    
  def __mul__(self, v):
    if isinstance(v, Vec3):
      return Vec3(self.x * v.x, self.y * v.y, self.z * v.z)
    else:
      return Vec3(self.x * v, self.y * v, self.z * v)
      
  def __div__(self, v):
    if isinstance(v, Vec3):
      return Vec3(self.x / v.x, self.y / v.y, self.z / v.z)
    else:
      return Vec3(self.x / v, self.y / v, self.z / v)
      
  def sq_length(self):
    return self.x * self.x + self.y * self.y + self.z * self.z
    
  def length(self):
    return math.sqrt(self.sq_length())
    
  def norm(self):
    return self / self.length()
    
  def dot(self, v):
    return self.x * v.x + self.y * v.y + self.z * v.z
    
  def lerp(self, v, i):
    return self + ( v - self ) * i
    
  @staticmethod
  def zero():
    return Vec3(0, 0, 0)
    
  @staticmethod
  def test():
    assert Vec3(1, 2, 3) + Vec3(3, 4, 5) == Vec3(4, 6, 8)
    assert Vec3(1, 2, 3) - Vec3(3, 4, 5) == Vec3(-2, -2, -2)
    assert Vec3(1, 2, 3) * Vec3(3, 4, 5) == Vec3(3, 8, 15)
    assert Vec3(1, 2, 3) * 3 == Vec3(3, 6, 9)
    assert Vec3(10, 15, 20) / Vec3(2, 5, 4) == Vec3(5, 3, 5)
    assert Vec3(10, 15, 20) / 5 == Vec3(2, 3, 4)
    assert feq(Vec3(3, 4, 5).sq_length(), 50)
    assert feq(Vec3(3, 4, 5).length(), math.sqrt(50))
    assert Vec3(0, -10, 0).norm() == Vec3(0, -1, 0)
    assert feq(Vec3(1, 2, 3).dot(Vec3(3, 4, 5)), 26)
    assert Vec3(0, 0, 0).lerp(Vec3(10, 20, -30), .5) == Vec3(5, 10, -15)
    
Vec3.test()

