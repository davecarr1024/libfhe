# -*- coding: utf-8 -*-
from Core.App import App
import sys

app = App(name = "App")

map(app.loadChild,sys.argv[1:])

app.run()
