# -*- coding: utf-8 -*-

@self.func(None,'VarMap')
def msg_buttonClicked(args):
    self.log("clicked")
    if args.get("path") == self.path:
        entity.getRoot().setVar("shutdown",True)
