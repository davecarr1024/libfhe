# -*- coding: utf-8 -*-

def addFunc(self):
    def _addFunc(tret, targ):
        def __addFunc(func):
            self._addFunc(tret,targ,func)
        return __addFunc
    return _addFunc
