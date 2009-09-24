# -*- coding: utf-8 -*-

@self.func(None,str)
def msg_buttonClicked(path):
    if path == self.path:
        self.getRoot().setVar("shutdown",True)
