#include "VarMap.h"

namespace gge
{
    
    VarMap::VarMap()
    {
    }
    
    bool VarMap::hasVarName( const std::string& name ) const
    {
        return m_vars.find(name) != m_vars.end() || onHasVar(name);
    }
    
    Var VarMap::getRawVar( const std::string& name, bool useOnGet ) const
    {
        Var val = useOnGet ? onGetVar(name) : Var();
        if ( !val.empty() )
        {
            return val;
        }
        else
        {
            std::map<std::string,Var>::const_iterator i = m_vars.find(name);
            return i != m_vars.end() ? i->second : Var();
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
}
