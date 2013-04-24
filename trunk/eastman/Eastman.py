#!/usr/bin/python

import curses
import time
import math

class RenderContext:
  def __init__( self, window ):
    self.window = window
    self.back = {}
    self.front = {}
    
  def putChar( self, x, y, c ):
    x, y = int( x ), int( y )
    if x >= 0 and x < self.window.width and y >= 0 and y < self.window.height and ( x != self.window.width - 1 or y != self.window.height - 1 ):
      self.back[x,y] = c
    
  def putString( self, x, y, s, align ):
    x, y = self.resolve( x, y )
    l = len( s )
    for i, c in enumerate( s ):
      self.putChar( x - l/2 + l/2 * align + i, y, c )
      
  def putLine( self, x0, y0, x1, y1, c ):
    x0, y0 = self.resolve( x0, y0 )
    x1, y1 = self.resolve( x1, y1 )
    l = max( abs( x1 - x0 ), abs( y1 - y0 ) )
    for i in range( l + 1 ):
      d = i / float( l )
      self.putChar( x0 + d * ( x1 - x0 ), y0 + d * ( y1 - y0 ), c )
      
  def putCircle( self, x, y, r, c ):
    x, y = self.resolve( x, y )
    rx, ry = self.resolve( r, r )
    l = 2 * max( rx, ry )
    for i in range( l ):
      t = math.pi * 2 * ( i / float( l ) )
      self.putChar( x + rx * math.cos( t ), y + ry * math.sin( t ), c )
      
  def putRect( self, x, y, w, h, c ):
    x, y = self.resolve( x, y )
    w, h = self.resolve( w, h )
    for i in range( w ):
      self.putChar( x + i, y, c )
      self.putChar( x + i, y + h - 1, c )
    for i in range( h ):
      self.putChar( x, y + i, c )
      self.putChar( x + w - 1, y + i, c )
      
  def renderChar( self, pos, c ):
    self.window.screen.addch( pos[1], pos[0], c )
    
  def render( self ):
    for pos, c in self.back.iteritems():
      if pos not in self.front or self.front[pos] != c:
        self.renderChar( pos, c )
    for pos, c in self.front.iteritems():
      if pos not in self.back:
        self.renderChar( pos, ' ' )
      
  def resolve( self, x, y ):
    if isinstance( x, float ):
      x *= self.window.width
    if isinstance( y, float ):
      y *= self.window.height
    return int( x ), int( y )
      
  def swap( self ):
    self.front = self.back
    self.back = {}

class Window:
  def __init__( self, root, dt = 0.01 ):
    self.root = root
    self.dt = dt

  def __enter__( self ):
    self.screen = curses.initscr()
    curses.cbreak()
    curses.noecho()
    self.screen.nodelay(1)
    self.screen.keypad(1)
    self.context = RenderContext( self )
    self.height, self.width = self.screen.getmaxyx()
    
    start = last = time.time()
    self.shutdown = False
    while not self.shutdown:
      now = time.time()
      if now - last >= self.dt:
        last += self.dt
        t = now - start
        
        c = self.screen.getch()
        if c != curses.ERR:
          self.root.input( c )
        
        self.root.update( self.dt, t )
        self.root.render( self.context )
        
        self.context.render()
        self.screen.refresh()
        self.context.swap()
  
  def __exit__( self, *args ):
    curses.nocbreak()
    self.screen.keypad(0)
    curses.echo()
    curses.endwin()  

class Object:
  def update( self, dt, t ):
    pass
    
  def render( self, context ):
    pass
    
  def input( self, c ):
    pass

class Eastman( Object ):
  def __init__( self ):
    self.c = None
    self.t = None
  
  def render( self, context ):
    context.putString( 0.5, 0.5, '%s %s' % ( self.t, self.c ), 0 )
    
  def update( self, dt, t ):
    self.t = t
    
  def input( self, c ):
    self.c = c
    
if __name__ == '__main__':
  with Window( Eastman() ) as window:
    pass
