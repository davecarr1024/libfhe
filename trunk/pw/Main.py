# -*- coding: utf-8 -*-
from Core.Node import Node
import sys

app = Node.buildNode(type = "Core.App", name = "App")

if len(sys.argv) > 1:
    map(app.addChild,map(Node.load,sys.argv[1:]))
else:
    app.addChild(Node.load("Eastman/Eastman.app"))

app.run()
