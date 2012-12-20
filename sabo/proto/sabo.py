from World import World
from Body import Body
from Vec2 import Vec2
import random

if __name__ == '__main__':
  bodies = [ Body.rectangle( Vec2( 0, -.5 ), Vec2( 1000, 1 ), True ) ]
  for i in range( 10 ):
    bodies.append( Body.rectangle( Vec2( random.uniform( -100, 100 ), random.uniform( 5, 50 ) ),
                                   Vec2( random.uniform( 1, 2 ), random.uniform( 1, 2 ) ) ) )
    bodies[-1].pointMasses[0].applyForce( Vec2( random.uniform( -100, 100 ), random.uniform( -100, 100 ) ) )
  World( bodies ).run()
