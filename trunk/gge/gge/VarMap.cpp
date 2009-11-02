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
        if ( name == "scale" ) printf("get scale\n");
        Var val = onGetVar(name);
        if ( !val.empty() )
        {
            if ( name == "scale" ) printf("return onGetVar val %s\n",val.toString().c_str());
            return val;
        }
        else
        {
            std::map<std::string,Var>::const_iterator i = m_vars.find(name);
            if ( name == "scale" ) printf("return m_vars val %s\n",(i != m_vars.end() ? i->second : Var()).toString().c_str());
            return i != m_vars.end() ? i->second : Var();
        }
    }
    
    void VarMap::setRawVar( const std::string& name, Var val )
    {
        if ( name == "scale" )
        {
            printf("scale = %s\n",val.toString().c_str());
        }
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
