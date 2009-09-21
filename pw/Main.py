# -*- coding: utf-8 -*-
from Core.Node import Node
import sys

app = Node.buildNode(type = "Core.App", name = "App")

map(app.addChild,map(Node.load,sys.argv[1:]))

app.run()
