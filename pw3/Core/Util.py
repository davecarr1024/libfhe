import yaml

def dynload(name):
    try:
        exec "%s import %s as c" % (name,name.split('.')[-1]) in locals()
        return c
    except:
        print "unable to dynload", name
        throw
        
def deepEval(val,**env):
    if isinstance(val,str):
        try:
            return deepEval(eval(val,env))
        except:
            return val
    elif isinstance(val,list):
        return map(deepEval,val)
    elif isinstance(val,dict):
        return dict(zip(val.keys(),map(deepEval,val.values())))
    else:
        return val

def deepLoad(filename):
    return deepEval(yaml.load(open(filename,'r')))

def save(filename, data):
    yaml.dump(open(filename,'w'),data)
