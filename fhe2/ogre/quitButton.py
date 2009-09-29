# -*- coding: utf-8 -*-

@self.func(None,VarMap)
def msg_buttonClicked(args):
    self.log("clicked")
    if args.getVar("path",None) == self.path:
        self.getRoot().setVar("shutdown",True)

self.publish("buttonClicked",VarMap())
