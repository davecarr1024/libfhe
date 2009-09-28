#include "FuncMap.h"
#include "Var.h"
#include "VarMap.h"

#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Vec3.h"
#include "math/Quat.h"

#include <cassert>

namespace fhe
{
    
    FuncMap::FuncMap()
    {
    }
    
    FuncMap::~FuncMap()
    {
        clearFuncs();
    }
    
    void FuncMap::clearFuncs()
    {
        m_funcs.clear();
    }
    
    void FuncMap::removeFunc( const std::string& name )
    {
        m_funcs.erase(name);
    }
    
    void FuncMap::addFunc( const std::string& name, AbstractFunc* func )
    {
        assert(func);
        removeFunc(name);
        m_funcs[name] = func;
    }
    
    bool FuncMap::pyHasFunc( const std::string& name )
    {
        return m_funcs.find(name) != m_funcs.end();
    }
    
    void FuncMap::pyAddFunc( const std::string& name, boost::python::object tret, boost::python::object targ, boost::python::object func )
    {
        if ( tret == boost::python::object() )
        {
            pyAddFuncWithRet<void>(name,targ,func);
        }
        else
        {
            std::string type = boost::python::extract<std::string>(tret.attr("__name__"));
            
            if ( type == "bool" )
            {
                pyAddFuncWithRet<bool>(name,targ,func);
            }
            else if ( type == "int" )
            {
                pyAddFuncWithRet<int>(name,targ,func);
            }
            else if ( type == "float" )
            {
                pyAddFuncWithRet<float>(name,targ,func);
            }
            else if ( type == "str" )
            {
                pyAddFuncWithRet<std::string>(name,targ,func);
            }
            else if ( type == "VarMap" )
            {
                pyAddFuncWithRet<VarMap>(name,targ,func);
            }
            else if ( type == "Var" )
            {
                pyAddFuncWithRet<Var>(name,targ,func);
            }
            else if ( type == "Vec2" )
            {
                pyAddFuncWithRet<Vec2>(name,targ,func);
            }
            else if ( type == "Rot" )
            {
                pyAddFuncWithRet<Rot>(name,targ,func);
            }
            else if ( type == "Vec3" )
            {
                pyAddFuncWithRet<Vec3>(name,targ,func);
            }
            else if ( type == "Quat" )
            {
                pyAddFuncWithRet<Quat>(name,targ,func);
            }
            else
            {
                throw std::runtime_error( "unable to bind ret type " + type + " for func " + name );
            }
        }
    }
    
    template <class TRet>
    void FuncMap::pyAddFuncWithRet( const std::string& name, boost::python::object targ, boost::python::object func )
    {
        if ( targ == boost::python::object() )
        {
            addFunc(name,new PyFunc<TRet,void>(func));
        }
        else
        {
            std::string type = boost::python::extract<std::string>(targ.attr("__name__"));
            
            if ( type == "bool" )
            {
                addFunc(name,new PyFunc<TRet,bool>(func));
            }
            else if ( type == "int" )
            {
                addFunc(name,new PyFunc<TRet,int>(func));
            }
            else if ( type == "float" )
            {
                addFunc(name,new PyFunc<TRet,float>(func));
            }
            else if ( type == "str" )
            {
                addFunc(name,new PyFunc<TRet,std::string>(func));
            }
            else if ( type == "Var" )
            {
                addFunc(name,new PyFunc<TRet,Var>(func));
            }
            else if ( type == "VarMap" )
            {
                addFunc(name,new PyFunc<TRet,VarMap>(func));
            }
            else if ( type == "Vec2" )
            {
                addFunc(name,new PyFunc<TRet,Vec2>(func));
            }
            else if ( type == "Rot" )
            {
                addFunc(name,new PyFunc<TRet,Rot>(func));
            }
            else if ( type == "Vec3" )
            {
                addFunc(name,new PyFunc<TRet,Vec3>(func));
            }
            else if ( type == "Quat" )
            {
                addFunc(name,new PyFunc<TRet,Quat>(func));
            }
            else
            {
                throw std::runtime_error("unable to bind arg type " + type + " for func " + name );
            }
        }
    }
    
    boost::python::object FuncMap::pyCallFunc( const std::string& name, boost::python::object arg )
    {
        if ( arg == boost::python::object() )
        {
            return pyCallFuncNoArg(name);
        }
        else
        {
            std::string type = boost::python::extract<std::string>(arg.attr("__class__").attr("__name__"));
            
            if ( type == "bool" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<bool>(arg)());
            }
            else if ( type == "int" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<int>(arg)());
            }
            else if ( type == "float" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<float>(arg)());
            }
            else if ( type == "str" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<std::string>(arg)());
            }
            else if ( type == "Var" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<Var>(arg)());
            }
            else if ( type == "VarMap" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<VarMap>(arg)());
            }
            else if ( type == "Vec2" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<Vec2>(arg)());
            }
            else if ( type == "Rot" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<Rot>(arg)());
            }
            else if ( type == "Vec3" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<Vec3>(arg)());
            }
            else if ( type == "Quat" )
            {
                return pyCallFuncWithArg(name,boost::python::extract<Quat>(arg)());
            }
            else
            {
                throw std::runtime_error("unable to bind arg type " + type + " to call func " + name );
            }
        }
    }
    
    boost::python::object FuncMap::pyCallFuncNoArg( const std::string& name )
    {
        if ( hasFunc<void,void>(name) )
        {
            callFunc<void>(name);
            return boost::python::object();
        }
        else if ( hasFunc<bool,void>(name) )
        {
            return boost::python::object(callFunc<bool>(name));
        }
        else if ( hasFunc<int,void>(name) )
        {
            return boost::python::object(callFunc<int>(name));
        }
        else if ( hasFunc<float,void>(name) )
        {
            return boost::python::object(callFunc<float>(name));
        }
        else if ( hasFunc<std::string,void>(name) )
        {
            return boost::python::object(callFunc<std::string>(name));
        }
        else if ( hasFunc<Var,void>(name) )
        {
            return boost::python::object(callFunc<Var>(name));
        }
        else if ( hasFunc<VarMap,void>(name) )
        {
            return boost::python::object(callFunc<VarMap>(name));
        }
        else if ( hasFunc<Vec2,void>(name) )
        {
            return boost::python::object(callFunc<Vec2>(name));
        }
        else if ( hasFunc<Rot,void>(name) )
        {
            return boost::python::object(callFunc<Rot>(name));
        }
        else if ( hasFunc<Vec3,void>(name) )
        {
            return boost::python::object(callFunc<Vec3>(name));
        }
        else if ( hasFunc<Quat,void>(name) )
        {
            return boost::python::object(callFunc<Quat>(name));
        }
        else
        {
            throw std::runtime_error("unable to find convertable ret type to call " + name );
        }
    }
    
    template <class TArg>
    boost::python::object FuncMap::pyCallFuncWithArg( const std::string& name, const TArg& arg )
    {
        if ( hasFunc<void,TArg>(name) )
        {
            callFunc<void,TArg>(name,arg);
            return boost::python::object();
        }
        else if ( hasFunc<bool,TArg>(name) )
        {
            return boost::python::object(callFunc<bool,TArg>(name,arg));
        }
        else if ( hasFunc<int,TArg>(name) )
        {
            return boost::python::object(callFunc<int,TArg>(name,arg));
        }
        else if ( hasFunc<float,TArg>(name) )
        {
            return boost::python::object(callFunc<float,TArg>(name,arg));
        }
        else if ( hasFunc<std::string,TArg>(name) )
        {
            return boost::python::object(callFunc<std::string,TArg>(name,arg));
        }
        else if ( hasFunc<Var,TArg>(name) )
        {
            return boost::python::object(callFunc<Var,TArg>(name,arg));
        }
        else if ( hasFunc<VarMap,TArg>(name) )
        {
            return boost::python::object(callFunc<VarMap,TArg>(name,arg));
        }
        else if ( hasFunc<Vec2,TArg>(name) )
        {
            return boost::python::object(callFunc<Vec2,TArg>(name,arg));
        }
        else if ( hasFunc<Rot,TArg>(name) )
        {
            return boost::python::object(callFunc<Rot,TArg>(name,arg));
        }
        else if ( hasFunc<Vec3,TArg>(name) )
        {
            return boost::python::object(callFunc<Vec3,TArg>(name,arg));
        }
        else if ( hasFunc<Quat,TArg>(name) )
        {
            return boost::python::object(callFunc<Quat,TArg>(name,arg));
        }
        else
        {
            throw std::runtime_error("unable to find convertable ret type to call " + name );
        }
    }
}
