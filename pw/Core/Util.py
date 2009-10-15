# -*- coding: utf-8 -*-
import yaml

def dynload(name):
    try:
        exec "from %s import %s as c" % (name,name.split('.')[-1]) in locals()
        return c
    except ImportError:
        return None

def flatten(l):
    if isinstance(l,list) or isinstance(l,tuple):
        return sum(map(flatten,l),[])
    elif isinstance(l,dict):
        return sum(map(flatten,l.values()),[])
    else:
        return [l]

def deepEval(d, env = None):
    if isinstance(d,dict):
        return dict([(key,deepEval(val,env)) for key, val in d.iteritems()])
    elif isinstance(d,list):
        return map(deepEval,d)
    elif isinstance(d,str):
        try:
            return deepEval(eval(d,env))
        except:
            return d
    else:
        return d

def evalLoad(filename, env = None):
    return deepEval(load(filename),env)
    
def load(filename):
    return yaml.load(open(filename))
    
def save(filename, data):
    yaml.dump(data,open(filename,'w'))
