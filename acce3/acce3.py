#!/usr/bin/python

from core.sim import Sim
from graphics.window import Window

if __name__ == '__main__':
  Sim('acce3',
    Window('window', title = 'acce3')
  ).call('run')

