#include "VarMap.h"

namespace gge
{
    
    GGE_TO_PYTHON_CONVERTER(VarMap,obj.toPy());
    GGE_FROM_PYTHON_CONVERTER(VarMap,VarMap::fromPy(obj));
    
    VarMap::VarMap()
    {
    }
    
    bool VarMap::hasVarName( const std::string& name ) const
    {
        return m_vars.find(name) != m_vars.end() || onHasVar(name);
    }
    
    Var VarMap::getRawVar( const std::string& name ) const
    {
        Var val = onGetVar(name);
        if ( !val.empty() )
        {
            return val;
        }
        else
        {
            std::map<std::string,Var>::const_iterator i = m_vars.find(name);
            assert(i != m_vars.end());
            return i->second;
        }
    }
    
    void VarMap::setRawVar( const std::string& name, Var val )
    {
        m_vars[name] = val;
        onSetVar(name,val);
    }
    
    Var VarMap::onGetVar( const std::string& name ) const
    {
        return Var();
    }
    
    void VarMap::onSetVar( const std::string& name, const Var& val )
    {
    }
    
    bool VarMap::onHasVar( const std::string& name ) const
    {
        return false;
    }
    
    boost::python::object VarMap::toPy() const
    {
        boost::python::dict d;
        for ( std::map<std::string,Var>::const_iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            d[i->first] = i->second.toPy();
        }
        return d;
    }
    
    VarMap VarMap::fromPy( boost::python::object obj )
    {
        VarMap varMap;
        
        boost::python::object items = obj.attr("items")();
        
        for ( int i = 0; i < boost::python::len(items); ++i )
        {
            varMap.m_vars[boost::python::extract<std::string>(items[i][0])()] = Var::fromPy(items[i][1]);
        }
        
        return varMap;
    }
    
}
