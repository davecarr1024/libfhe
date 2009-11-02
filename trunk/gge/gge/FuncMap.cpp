#include "FuncMap.h"
#include "Var.h"
#include "VarMap.h"

namespace gge
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
        for ( std::map<std::string,AbstractFunc*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
        {
            delete i->second;
        }
        m_funcs.clear();
    }
    
    bool FuncMap::hasFuncName( const std::string& name )
    {
        return m_funcs.find(name) != m_funcs.end();
    }
    
    void FuncMap::addFunc( const std::string& name, AbstractFunc* func )
    {
        if ( hasFuncName( name ) )
        {
            delete m_funcs[name];
        }
        m_funcs[name] = func;
    }
    
    FuncMap::PyAddFunc::PyAddFunc( FuncMap* funcMap, boost::python::object tret, boost::python::object targ ) :
        m_funcMap(funcMap),
        m_tret(tret),
        m_targ(targ)
    {
    }
    
    void FuncMap::PyAddFunc::call( boost::python::object func )
    {
        if ( m_tret == boost::python::object() )
        {
            bind<void>(func);
        }
        else
        {
            boost::python::extract<std::string> e(m_tret);
            std::string type = e.check() ? e() : boost::python::extract<std::string>(m_tret.attr("__name__"));
            
            if ( type == "bool" )
            {
                bind<bool>(func);
            }
            else if ( type == "int" )
            {
                bind<int>(func);
            }
            else if ( type == "float" )
            {
                bind<float>(func);
            }
            else if ( type == "str" )
            {
                bind<std::string>(func);
            }
            else if ( type == "Var" )
            {
                bind<Var>(func);
            }
            else if ( type == "VarMap" )
            {
                bind<VarMap>(func);
            }
            else
            {
                throw std::runtime_error("unable to bind python return type " + type);
            }
        }
    }
    
    template <class TRet>
    void FuncMap::PyAddFunc::bind( boost::python::object func )
    {
        std::string name = boost::python::extract<std::string>(func.attr("__name__"));
        
        if ( m_targ == boost::python::object() )
        {
            m_funcMap->addFunc( name, new PyFunc<TRet,void>(func) );
        }
        else
        {
            boost::python::extract<std::string> e(m_targ);
            std::string type = e.check() ? e() : boost::python::extract<std::string>(m_targ.attr("__name__"));
            
            if ( type == "bool" )
            {
                m_funcMap->addFunc( name, new PyFunc<TRet,bool>(func) );
            }
            else if ( type == "int" )
            {
                m_funcMap->addFunc( name, new PyFunc<TRet,int>(func) );
            }
            else if ( type == "float" )
            {
                m_funcMap->addFunc( name, new PyFunc<TRet,float>(func) );
            }
            else if ( type == "str" )
            {
                m_funcMap->addFunc( name, new PyFunc<TRet,std::string>(func) );
            }
            else if ( type == "Var" )
            {
                m_funcMap->addFunc( name, new PyFunc<TRet,Var>(func) );
            }
            else if ( type == "VarMap" )
            {
                m_funcMap->addFunc( name, new PyFunc<TRet,VarMap>(func) );
            }
            else
            {
                throw std::runtime_error("unable to bind python arg type " + type );
            }
        }
    }
    
    FuncMap::PyCall::PyCall( AbstractFunc* func ) :
        m_func(func)
    {
    }
    
    boost::python::object FuncMap::PyCall::call( boost::python::object arg )
    {
        return m_func->pyCall(arg);
    }
    
    boost::python::object FuncMap::PyCall::callNoArg()
    {
        return call(boost::python::object());
    }
    
    boost::python::object FuncMap::defineClass()
    {
        boost::python::class_<PyAddFunc>("PyAddFunc",boost::python::no_init)
            .def("__call__",&PyAddFunc::call)
        ;
        boost::python::class_<PyCall>("PyCall",boost::python::no_init)
            .def("__call__",&PyCall::call)
            .def("__call__",&PyCall::callNoArg)
        ;
        return boost::python::object();
    }
    
    boost::python::object FuncMap::pyAddFunc( boost::python::object tret, boost::python::object targ )
    {
        return boost::python::object( PyAddFunc(this,tret,targ) );
    }
    
    boost::python::object FuncMap::pyGetFunc( const std::string& name )
    {
        assert(hasFuncName(name));
        return boost::python::object( PyCall(m_funcs[name]) );
    }
}
