#include "VarMap.h"
#include <cassert>
#include <stdexcept>

namespace fhe
{
    
    VarMap::VarMap()
    {
    }
    
    VarMap::VarMap( const VarMap& varMap )
    {
        for ( std::map<std::string, Var>::const_iterator i = varMap.m_vars.begin(); i != varMap.m_vars.end(); ++i )
        {
            m_vars[i->first] = Var(i->second);
        }
    }
    
    VarMap& VarMap::operator=( const VarMap& varMap )
    {
        clearVars();
        
        for ( std::map<std::string, Var>::const_iterator i = varMap.m_vars.begin(); i != varMap.m_vars.end(); ++i )
        {
            m_vars[i->first] = Var(i->second);
        }
    }
    
    VarMap::VarMap( boost::python::object obj )
    {
        *this = VarMap::fromPy(obj);
    }
    
    void VarMap::clearVars()
    {
        m_vars.clear();
    }
    
    void VarMap::removeVar( const std::string& name )
    {
        m_vars.erase( name );
    }
    
    template <class T>
    bool VarMap::hasVar( const std::string& name )
    {
        return m_vars.find(name) != m_vars.end() && m_vars[name].is<T>();
    }
    
    template <class T>
    T VarMap::getVar( const std::string& name )
    {
        onGetVar(name);
        assert(hasVar<T>(name));
        return m_vars[name].get<T>();
    }
    
    template <class T>
    T VarMap::getVar( const std::string& name, const T& def )
    {
        onGetVar(name);
        if ( hasVar<T>(name) )
        {
            return m_vars[name].get<T>();
        }
        else
        {
            return def;
        }
    }
    
    template <class T>
    void VarMap::setVar( const std::string& name, const T& val )
    {
        if ( m_vars.find(name) == m_vars.end() )
        {
            m_vars[name] = Var();
        }
        m_vars[name].set<T>(val);
        onSetVar(name,m_vars[name]);
    }
    
    bool VarMap::pyHasVar( const std::string& name )
    {
        return m_vars.find(name) != m_vars.end();
    }
    
    boost::python::object VarMap::pyGetVar( const std::string& name )
    {
        onGetVar(name);
        assert(pyHasVar(name));
        return m_vars[name].toPy();
    }
    
    boost::python::object VarMap::pyGetVarDef( const std::string& name, boost::python::object def )
    {
        onGetVar(name);
        if ( pyHasVar(name) )
        {
            return m_vars[name].toPy();
        }
        else
        {
            return def;
        }
    }
    
    void VarMap::pySetVar( const std::string& name, boost::python::object val )
    {
        m_vars[name] = Var::fromPy(val);
        onSetVar(name,m_vars[name]);
    }
    
    boost::python::object VarMap::toPy()
    {
        boost::python::dict dict;
        
        for ( std::map<std::string, Var>::iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            dict[i->first] = i->second.toPy();
        }
        
        return dict;
    }
    
    VarMap VarMap::fromPy( boost::python::object obj )
    {
        std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
        
        if ( type == "VarMap" )
        {
            return boost::python::extract<VarMap>(obj);
        }
        else if ( type == "dict" )
        {
            VarMap varMap;
            boost::python::object items = obj.attr("items")();
            for ( int i = 0; i < boost::python::len(items); ++i )
            {
                varMap.pySetVar( boost::python::extract<std::string>(items[i][0]), items[i][1] );
            }
            return varMap;
        }
        else
        {
            throw std::runtime_error( "can't create VarMap from python type " + type );
        }
    }
    
    boost::python::object VarMap::defineClass()
    {
        return boost::python::class_<VarMap>("VarMap", boost::python::init<>())
            .def(boost::python::init<boost::python::dict>())
            .def("hasVar",&VarMap::pyHasVar)
            .def("getVar",&VarMap::pyGetVar)
            .def("getVar",&VarMap::pyGetVarDef)
            .def("setVar",&VarMap::pySetVar)
            .def("removeVar",&VarMap::removeVar)
            .def("clearVars",&VarMap::clearVars)
        ;
    }
}
