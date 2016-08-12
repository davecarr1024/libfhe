from node import Node
from time import time as sys_time, sleep

class Sim(Node):
  def __init__(self, name, *children, **args):
    Node.__init__(self, name, *children, **args)
    
  def run(self):
    try:
      self.visit_call('sim_start')
      last_time = start_time = sys_time()
      while not any(self.aggregate_visit_call('get', 'shutdown')):
        time = sys_time()
        self.visit_call('sim_tick', time - start_time, time - last_time)
        last_time = time
    finally:
      self.visit_call('sim_end')

