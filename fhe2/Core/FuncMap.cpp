#include "FuncMap.h"
#include "Var.h"
#include "VarMap.h"
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
    
    template <class TRet, class TArg>
    bool FuncMap::hasFunc( const std::string& name )
    {
        return m_funcs.find(name) != m_funcs.end() && m_funcs[name]->cast<TRet,TArg>();
    }
    
    void FuncMap::addFunc( const std::string& name, AbstractFunc* func )
    {
        assert(func);
        removeFunc(name);
        m_funcs[name] = func;
    }
    
    template <class TRet, class TArg>
    TRet FuncMap::call( const std::string& name, const TArg& arg )
    {
        bool hasThisFunc = hasFunc<TRet,TArg>(name);
        assert(hasThisFunc);
        return m_funcs[name]->cast<TRet,TArg>()->call(arg);
    }
    
    template <class TRet>
    TRet FuncMap::call( const std::string& name )
    {
        bool hasThisFunc = hasFunc<TRet,void>(name);
        assert(hasThisFunc);
        return m_funcs[name]->cast<TRet,void>()->call();
    }
    
    template <class TObj, class TRet, class TArg>
    void FuncMap::addFunc( const std::string& name, TRet (TObj::*method)(TArg), TObj* obj )
    {
        addFunc( name, new Func<TRet,TArg>(boost::bind(method,obj,_1)) );
    }
    
    template <class TObj, class TRet>
    void FuncMap::addFunc( const std::string& name, TRet (TObj::*method)(), TObj* obj )
    {
        addFunc( name, new Func<TRet,void>(boost::bind(method,obj,_1)) );
    }
    
    void FuncMap::pyAddFunc( const std::string& name, boost::python::object tret, 
        boost::python::object targ, boost::python::object func )
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
            else if ( type == "Var" )
            {
                pyAddFuncWithRet<Var>(name,targ,func);
            }
            else if ( type == "VarMap" )
            {
                pyAddFuncWithRet<VarMap>(name,targ,func);
            }
            else
            {
                throw std::runtime_error("unable to bind ret type " + type + " for func " + name );
            }
        }
    }
    
    template <class TRet>
    void FuncMap::pyAddFuncWithRet( const std::string& name, boost::python::object targ, boost::python::object func )
    {
        if ( targ == boost::python::object() )
        {
            addFunc(name, new PyFunc<TRet,void>(func) );
        }
        else
        {
            std::string type = boost::python::extract<std::string>(targ.attr("__name__"));
            
            if ( type == "bool" )
            {
                addFunc(name, new PyFunc<TRet,bool>(func) );
            }
            else if ( type == "int" )
            {
                addFunc(name, new PyFunc<TRet,int>(func) );
            }
            else if ( type == "float" )
            {
                addFunc(name, new PyFunc<TRet,float>(func) );
            }
            else if ( type == "str" )
            {
                addFunc(name, new PyFunc<TRet,std::string>(func) );
            }
            else if ( type == "Var" )
            {
                addFunc(name, new PyFunc<TRet,Var>(func) );
            }
            else if ( type == "VarMap" )
            {
                addFunc(name, new PyFunc<TRet,VarMap>(func) );
            }
            else
            {
                throw std::runtime_error( "unable to bind arg type " + type + " for func " + name );
            }
        }
    }
    
    boost::python::object FuncMap::pyCall( const std::string& name, boost::python::object arg )
    {
        if ( arg == boost::python::object() )
        {
            return pyCallWithNoArg( name );
        }
        else
        {
            std::string type = boost::python::extract<std::string>(arg.attr("__class__").attr("__name__"));
            
            if ( type == "bool" )
            {
                return pyCallWithArg<bool>(name, boost::python::extract<bool>(arg));
            }
            else if ( type == "int" )
            {
                return pyCallWithArg<int>(name, boost::python::extract<int>(arg));
            }
            else if ( type == "float" )
            {
                return pyCallWithArg<float>(name, boost::python::extract<float>(arg));
            }
            else if ( type == "str" )
            {
                return pyCallWithArg<std::string>(name, boost::python::extract<std::string>(arg));
            }
            else if ( type == "Var" )
            {
                return pyCallWithArg<Var>(name, boost::python::extract<Var>(arg));
            }
            else if ( type == "VarMap" )
            {
                return pyCallWithArg<VarMap>(name, boost::python::extract<VarMap>(arg));
            }
            else
            {
                throw std::runtime_error( "unable to bind arg type " + type + " to call func " + name );
            }
        }
    }
    
    boost::python::object FuncMap::pyCallWithNoArg( const std::string& name )
    {
        if ( hasFunc<void,void>(name) )
        {
            call<void>(name);
            return boost::python::object();
        }
        else if ( hasFunc<bool,void>(name) )
        {
            return boost::python::object(call<bool>(name));
        }
        else if ( hasFunc<int,void>(name) )
        {
            return boost::python::object(call<int>(name));
        }
        else if ( hasFunc<float,void>(name) )
        {
            return boost::python::object(call<float>(name));
        }
        else if ( hasFunc<std::string,void>(name) )
        {
            return boost::python::object(call<std::string>(name));
        }
        else if ( hasFunc<Var,void>(name) )
        {
            return boost::python::object(call<Var>(name));
        }
        else if ( hasFunc<VarMap,void>(name) )
        {
            return boost::python::object(call<VarMap>(name));
        }
        else
        {
            throw std::runtime_error( "no convertable function for " + name );
        }
    }
    
    template <class TArg>
    boost::python::object FuncMap::pyCallWithArg( const std::string& name, const TArg& arg )
    {
        if ( hasFunc<void,TArg>(name) )
        {
            call<void,TArg>(name, arg);
            return boost::python::object();
        }
        else if ( hasFunc<bool,TArg>(name) )
        {
            return boost::python::object(call<bool,TArg>(name,arg));
        }
        else if ( hasFunc<int,TArg>(name) )
        {
            return boost::python::object(call<int,TArg>(name,arg));
        }
        else if ( hasFunc<float,TArg>(name) )
        {
            return boost::python::object(call<float,TArg>(name,arg));
        }
        else if ( hasFunc<std::string,TArg>(name) )
        {
            return boost::python::object(call<std::string,TArg>(name,arg));
        }
        else if ( hasFunc<Var,TArg>(name) )
        {
            return boost::python::object(call<Var,TArg>(name,arg));
        }
        else if ( hasFunc<VarMap,TArg>(name) )
        {
            return boost::python::object(call<VarMap,TArg>(name,arg));
        }
        else
        {
            throw std::runtime_error( "no convertable function for " + name );
        }
    }
    
    bool FuncMap::pyHasFunc( const std::string& name, boost::python::object tret, boost::python::object targ )
    {
        if ( tret == boost::python::object() )
        {
            return pyHasFuncWithRet<void>(name,targ);
        }
        else
        {
            std::string type = boost::python::extract<std::string>(tret.attr("__name__"));
            
            if ( type == "bool" )
            {
                return pyHasFuncWithRet<bool>(name,targ);
            }
            else if ( type == "int" )
            {
                return pyHasFuncWithRet<int>(name,targ);
            }
            else if ( type == "float" )
            {
                return pyHasFuncWithRet<float>(name,targ);
            }
            else if ( type == "str" )
            {
                return pyHasFuncWithRet<std::string>(name,targ);
            }
            else if ( type == "Var" )
            {
                return pyHasFuncWithRet<Var>(name,targ);
            }
            else if ( type == "VarMap" )
            {
                return pyHasFuncWithRet<VarMap>(name,targ);
            }
            else
            {
                throw std::runtime_error( "can't get hasFunc for unknown ret type " + type );
            }
        }
    }
    
    template <class TRet>
    bool FuncMap::pyHasFuncWithRet( const std::string& name, boost::python::object targ )
    {
        if ( targ == boost::python::object() )
        {
            return hasFunc<TRet,void>(name);
        }
        else
        {
            std::string type = boost::python::extract<std::string>(targ.attr("__name__"));
            
            if ( type == "bool" )
            {
                return hasFunc<TRet,bool>(name);
            }
            else if ( type == "int" )
            {
                return hasFunc<TRet,int>(name);
            }
            else if ( type == "float" )
            {
                return hasFunc<TRet,float>(name);
            }
            else if ( type == "str" )
            {
                return hasFunc<TRet,std::string>(name);
            }
            else if ( type == "Var" )
            {
                return hasFunc<TRet,Var>(name);
            }
            else if ( type == "VarMap" )
            {
                return hasFunc<TRet,VarMap>(name);
            }
            else
            {
                throw std::runtime_error( "can't get hasFunc for unknown arg type " + type );
            }
        }
    }
    
    boost::python::object FuncMap::defineClass()
    {
        return boost::python::class_<FuncMap>("FuncMap",boost::python::init<>())
            .def("hasFunc",&FuncMap::pyHasFunc)
            .def("addFunc",&FuncMap::pyAddFunc)
            .def("call",&FuncMap::pyCall)
        ;
    }
}
