from vec2 import Vec2

class BB:
    def __init__(self, min = None, max = None):
        self.min = min or Vec2()
        self.max = max or Vec2(self.min.x,self.min.y)

    def __repr__(self):
        return "<BB %s %s>" % (self.min,self.max)

    def contains(self, v):
        if isinstance(v,Vec2):
            return v.x > self.min.x and v.x < self.max.x and v.y > self.min.y and v.y < self.max.y
        elif isinstance(v,BB):
            return v.min.x > self.min.x and v.max.x < self.max.x and v.min.y > self.min.y and v.max.y < self.max.y
        else:
            raise TypeError

    def overlaps(self, b):
        return b.min.x < self.max.x and b.max.x > self.min.x and b.min.y < self.max.y and b.max.y > self.min.y

    def expand(self, b):
        if isinstance(b,Vec2):
            self.min.x = min(self.min.x,b.x)
            self.min.y = min(self.min.y,b.y)
            self.max.x = max(self.max.x,b.x)
            self.max.y = max(self.max.y,b.y)
        elif isinstance(b,BB):
            self.min.x = min(self.min.x,b.min.x)
            self.min.y = min(self.min.y,b.min.y)
            self.max.x = max(self.max.x,b.max.x)
            self.max.y = max(self.max.y,b.max.y)
        else:
            raise TypeError

    def above(self, bb):
        return self.min.y > bb.max.y

    def below(self, bb):
        return self.max.y < bb.min.y

    def right(self, bb):
        return self.min.x > bb.max.x

    def left(self, bb):
        return self.max.x < bb.min.x

    def getCorners(self):
        return self.min, Vec2(self.max.x,self.min.y), self.max, Vec2(self.min.x,self.max.y)

    def getSize(self):
        return self.max - self.min

    def getCenter(self):
        return (self.min + self.max) / 2.0
