# -*- coding: utf-8 -*-

import yaml

def deepEval(v, *env):
    if isinstance(v,list):
        return sum([deepEval(i,*env) for i in v],[])
    elif isinstance(v,dict):
        return dict([(key,deepEval(val,*env)) for key,val in v.iteritems()])
    elif isinstance(v,str):
        try:
            return deepEval(eval(v,dict([(i.__name__,i) for i in env])),*env)
        except:
            return v
    else:
        return v
        
def deepLoad(filename, *env):
    return deepEval(yaml.load(open(filename,'r')),*env)
    
def dynload(name):
    exec 'from %s import %s as c' % (name,name.split('.')[-1]) in locals()
    return c

def flatten(v):
    if isinstance(v,list) or isinstance(v,tuple):
        r = []
        map(r.extend,map(flatten,v))
        return r
    else:
        return [v]
