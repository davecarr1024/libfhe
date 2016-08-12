from core.node import Node
from core.visitor import Visitor
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

class Window(Node):
  def __init__(self, name, *children, **args):
    Node.__init__(self, name, *children, **args)
    self.key_names = {}
    self.key_mods = {}
    for name in dir(pygame):
      if name.startswith('K_'):
        self.key_names[getattr(pygame, name)] = name[2:].lower()
      elif name.startswith('KMOD_'):
        self.key_mods[getattr(pygame, name)] = name[2:].lower()
    self.last_frame_time = 0
    self.fps = 60
    self.default_res = (800, 600)
    
  def sim_start(self):
    pygame.init()
    glutInit(())
    flags = pygame.OPENGL | pygame.DOUBLEBUF | pygame.HWSURFACE
    if self['fullscreen']:
      flags |= pygame.FULLSCREEN
    self.screen = pygame.display.set_mode(self.get('res', self.default_res), flags)
    pygame.display.set_caption(self.get('title', ''))
    glEnable(GL_TEXTURE_2D)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glLineWidth(2)
    glClearColor(*(self.get('clear_color', (1,1,1)) + (1,)))
    
  def sim_end(self):
    pygame.quit()
    
  def sim_tick(self, time, dtime):
    for event in pygame.event.get():
      event_name = pygame.event.event_name(event.type)
      self.call('event_%s' % event_name, event)
      
    frame_time = 1. / self.fps
    if time - self.last_frame_time > frame_time:
      self.last_frame_time += frame_time
      
      glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
      
      self.set_2d_projection()
      self.visit_call('render_2d')
      
      glFlush()
      pygame.display.flip()
      
  def set_2d_projection(self):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    res = self.get('res', self.default_res)
    gluOrtho2D(0, res[0], 0, res[1])

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslate(0, res[1], 0)
    glScalef(1, -1, 1)

    glColor(1, 1, 1, 1)
    glBindTexture(GL_TEXTURE_2D, 0)

    #glDisable(GL_LIGHTING)
    glDisable(GL_DEPTH_TEST)
    glDisable(GL_NORMALIZE)      

  def event_Quit(self, event):
    self.shutdown = True
    
  def key_info(self, event):
    name = self.key_names[event.key]
    mods = [name for mod, name in self.key_mods.iteritems() if mod & event.mod]
    return name, mods
    
  def event_KeyDown(self, event):
    if event.key == pygame.K_ESCAPE:
      self.shutdown = True
    name, mods = self.key_info(event)
    self.visit_call('key_down', name, mods)

  def event_KeyUp(self, event):
    name, mods = self.key_info(event)
    self.visit_call('key_up', name, mods)

