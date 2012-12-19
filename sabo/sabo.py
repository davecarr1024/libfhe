from World import World
from Body import Body
from Vec2 import Vec2

if __name__ == '__main__':
  World( [ Body.rectangle( Vec2( 0, 0 ), Vec2( 10, 1 ), True ), 
           Body.rectangle( Vec2( 0, 10 ), Vec2( 1, 1 ) ) 
           ]
       ).run()
