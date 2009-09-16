#!/usr/bin/env python

from core.app import App
from core.fileServer import fileServer
import sys

app = App()

map(app.load,filter(fileServer.isFile,sys.argv[1:]))

app.run()

app.save('save.sim')
