# -*- coding: utf-8 -*-

import yaml

def deepEval(s, **env):
    if isinstance(s,str):
        try:
            return deepEval(eval(s,env))
        except:
            return s
    elif isinstance(s,list):
        return map(deepEval,s)
    elif isinstance(s,dict):
        return dict(zip(s.keys(),map(deepEval,s.values())))
    else:
        return s
        
def deepLoad(filename, **env):
    return deepEval(yaml.load(open(filename,'r')),**env)
    
def dynLoad(name):
    try:
        exec "from %s import %s as c" % (name,name[name.rfind('.')+1:]) in locals()
        return c
    except:
        print "unable to dynload %s" % name
        raise

def save(data, filename):
    yaml.dump(data,open(filename,'w'))
