# -*- coding: utf-8 -*-

import sys
from Core.Entity import Entity

root = Entity(name = "Root")
app = root.buildAspect("Core.App")

map(root.loadChild,sys.argv[1:])

app.run(3)
