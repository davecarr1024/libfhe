# -*- coding: utf-8 -*-

from Simp.Sim import Sim
from Simp import Util

import sys

args = {}
map(args.update,map(Util.deepLoad,sys.argv[1:]))
sim = Sim(**args)
sim.run(2)
sim.save('dump')
