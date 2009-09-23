#include "VarMap.h"
#include <cassert>

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
        for ( std::map<std::string, Var>::const_iterator i = varMap.m_vars.begin(); i != varMap.m_vars.end(); ++i )
        {
            m_vars[i->first] = Var(i->second);
        }
        return *this;
    }
    
    VarMap::VarMap( boost::python::object obj )
    {
        *this = VarMap::fromPy( obj );
    }

    void VarMap::removeVar( const std::string& name )
    {
        m_vars.erase(name);
    }
    
    std::vector<std::string> VarMap::getVarNames()
    {
        std::vector<std::string> names;
        for ( std::map<std::string,Var>::iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            names.push_back( i->first );
        }
        return names;
    }
    
    bool VarMap::pyHasVar( const std::string& name )
    {
        return m_vars.find(name) != m_vars.end();
    }
    
    boost::python::object VarMap::pyGetVar( const std::string& name )
    {
        assert( pyHasVar( name ) );
        return m_vars[name].toPy();
    }
    
    boost::python::object VarMap::pyGetVarDef( const std::string& name, boost::python::object def )
    {
        if ( pyHasVar( name ) )
        {
            return pyGetVar( name );
        }
        else
        {
            return def;
        }
    }
    
    void VarMap::pySetVar( const std::string& name, boost::python::object val )
    {
        m_vars[name] = Var::fromPy( val );
    }
    
    boost::python::object VarMap::toPy()
    {
        boost::python::dict dict;
        
        for ( std::map<std::string,Var>::iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            dict[i->first] = i->second.toPy();
        }
        
        return dict;
    }
    
    VarMap VarMap::fromPy( boost::python::object obj )
    {
        VarMap varMap;

        std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
        
        if ( type == "dict" )
        {
            boost::python::object items = obj.attr("items")();

            for ( int i = 0; i < boost::python::len( items ); ++i )
            {
                varMap.m_vars[boost::python::extract<std::string>(items[i][0])] = Var::fromPy( items[i][1] );
            }
        }
        else if ( type == "VarMap" )
        {
            varMap = boost::python::extract<VarMap>(obj);
        }
        else
        {
            throw std::runtime_error("invalid type to build VarMap: " + type);
        }
        
        return varMap;
    }
    
    boost::python::object VarMap::defineClass()
    {
        return boost::python::class_<VarMap>("VarMap",boost::python::init<>())
            .def(boost::python::init<boost::python::dict>())
            .def("__contains__",&VarMap::pyHasVar)
            .def("__setitem__",&VarMap::pySetVar)
            .def("__getitem__",&VarMap::pyGetVar)
        ;
    }
}
