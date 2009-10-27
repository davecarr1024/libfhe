# -*- coding: utf-8 -*-

from Simp.Sim import Sim

sim = Sim.load('test.sim')

testEnt = sim.getEntity('testEnt')
assert testEnt

assert testEnt.getVar('i') == 23

assert testEnt.hasFunc('double')
assert testEnt.call('double',2) == 4
