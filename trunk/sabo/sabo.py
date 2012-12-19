from World import World
from Body import Body
from Vec2 import Vec2
import random

if __name__ == '__main__':
  bodies = [ Body.rectangle( Vec2( 0, -50 ), Vec2( 200, 100 ), True ) ]
  for i in range( 10 ):
    bodies.append( Body.rectangle( Vec2( 0, i + 0.5 ), Vec2( 1 + 1 * i, 1 ) ) )
    # bodies[-1].pointMasses[0].applyForce( Vec2( 10, 10 ) )
  World( bodies ).run()
