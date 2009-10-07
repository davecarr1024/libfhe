#include "FuncMap.h"

#include <cassert>
#include <stdexcept>

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
    
    bool FuncMap::hasFuncName( const std::string& name )
    {
        return m_funcs.find(name) != m_funcs.end();
    }
    
    void FuncMap::addFunc( const std::string& name, AbstractFunc* func )
    {
        assert(func);
        removeFunc(name);
        m_funcs[name] = func;
    }
    
    FuncMap::PyCall::PyCall( const std::string& name, AbstractFunc* func ) :
        m_name(name),
        m_func(func)
    {
        assert(m_func);
    }
    
    boost::python::object FuncMap::PyCall::call( boost::python::object arg )
    {
        return m_func->pyCall(arg);
    }
    
    boost::python::object FuncMap::PyCall::callNoArg()
    {
        return m_func->pyCall(boost::python::object());
    }
    
    std::string FuncMap::PyCall::repr()
    {
        return "<FuncMap::PyCall " + m_name + ">";
    }
    
    boost::python::object FuncMap::PyCall::defineClass()
    {
        return boost::python::class_<PyCall>("PyCall",boost::python::no_init)
            .def("__call__",&PyCall::call)
            .def("__call__",&PyCall::callNoArg)
            .def("__repr__",&PyCall::repr)
        ;
    }
    
    FuncMap::PyCall FuncMap::pyGetFunc( const std::string& name )
    {
        assert(m_funcs.find(name) != m_funcs.end());
        return PyCall(name,m_funcs[name]);
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
            else
            {
                throw std::runtime_error("can't bind unknown ret type " + type + " for func " + name);
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
            else
            {
                throw std::runtime_error("can't bind unknown arg type " + type + " for func " + name);
            }
        }
    }
    
    FuncMap::FuncClosure::FuncClosure( FuncMap* map, boost::python::object tret, boost::python::object targ ) :
        m_map(map),
        m_tret(tret),
        m_targ(targ)
    {
        assert(m_map);
    }
    
    void FuncMap::FuncClosure::call( boost::python::object func )
    {
        std::string name = boost::python::extract<std::string>(func.attr("__name__"));
        m_map->pyAddFunc( name, m_tret, m_targ, func );
    }
    
    boost::python::object FuncMap::FuncClosure::defineClass()
    {
        return boost::python::class_<FuncMap::FuncClosure>("FuncClosure",boost::python::no_init)
            .def("__call__",&FuncMap::FuncClosure::call)
        ;
    }
}
