from util import feq
from vec2 import Vec2
from vec3 import Vec3
import math

class Mat3:
  def __init__(self, *v):
    self.v = list(map(float,v)) or [0.] * 9
    assert len(self.v) == 9
    
  def __getitem__(self, i):
    return self.v[i[0] * 3 + i[1]]
    
  def __setitem__(self, i, v):
    self.v[i[0] * 3 + i[1]] = v
    
  def __repr__(self):
    return 'Mat3(%s)' % ', '.join(map(str, self.v))
    
  def __mul__(self, m):
    if isinstance(m, Mat3):
      res = Mat3()
      for i in range(3):
        for j in range(3):
          for k in range(3):
            res[i,j] += self[i,k] * m[k,j]
      return res
    elif isinstance(m, Vec2):
      v = self * Vec3(m.x, m.y, 1)
      return Vec2(v.x, v.y)
    elif isinstance(m, Vec3):
      v = Vec3.zero()
      for i in range(3):
        for j in range(3):
          v[i] += self[i,j] * m[j]
      return v
    else:
      raise TypeError(type(m))
      
  @staticmethod
  def identity():
    return Mat3(1, 0, 0, 0, 1, 0, 0, 0, 1)
    
  @staticmethod
  def translation(v):
    return Mat3(1, 0, v.x, 0, 1, v.y, 0, 0, 1)
    
  @staticmethod
  def scale(v):
    return Mat3(v.x, 0, 0, 0, v.y, 0, 0, 0, 1)
    
  @staticmethod
  def rotation(a):
    sa = math.sin(-a)
    ca = math.cos(-a)
    return Mat3(ca, sa, 0, -sa, ca, 0, 0, 0, 1)
    
  def det(self):
      return self.v[0] * (self.v[4] * self.v[8] - self.v[7] * self.v[5]) -\
             self.v[1] * (self.v[3] * self.v[8] - self.v[6] * self.v[5]) +\
             self.v[2] * (self.v[3] * self.v[7] - self.v[6] * self.v[4])

  def invert(self):
      det = self.det()
      return Mat3(
         (self.v[4] * self.v[8] - self.v[5] * self.v[7]) / det,
        -(self.v[1] * self.v[8] - self.v[7] * self.v[2]) / det,
         (self.v[1] * self.v[5] - self.v[4] * self.v[2]) / det,
        -(self.v[3] * self.v[8] - self.v[5] * self.v[6]) / det,
         (self.v[0] * self.v[8] - self.v[6] * self.v[2]) / det,
        -(self.v[0] * self.v[5] - self.v[3] * self.v[2]) / det,
         (self.v[3] * self.v[7] - self.v[6] * self.v[4]) / det,
        -(self.v[0] * self.v[7] - self.v[6] * self.v[1]) / det,
         (self.v[0] * self.v[4] - self.v[1] * self.v[3]) / det,
      )
        
  @staticmethod
  def test():
    v = Vec2(1,2)
    t = Vec2(50, -10)
    s = Vec2(-7, 3)
    r = math.pi / 2
    
    mi = Mat3.identity()
    mt = Mat3.translation(t)
    ms = Mat3.scale(s)
    mr = Mat3.rotation(r)
    
    assert v == mi * v
    assert v + t == mt * v
    assert v * s == ms * v
    assert v.rotate(r) == mr * v
    assert ms * ( mt * ( mr * v) ) == ( ms * mt * mr ) * v
    assert v == mi.invert() * v
    assert v - t == mt.invert() * v
    assert v / s == ms.invert() * v
    assert v.rotate(-r) == mr.invert() * v
    
Mat3.test()

