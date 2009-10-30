#include "VarMap.h"

namespace gge
{
    
    VarMap::VarMap()
    {
    }
    
    bool VarMap::hasVarName( const std::string& name ) const
    {
        return m_vars.find(name) != m_vars.end();
    }
    
    Var VarMap::onGetVar( const std::string& name )
    {
        return Var();
    }
    
    void VarMap::onSetVar( const std::string& name, const Var& val )
    {
    }
    
    bool VarMap::onHasVar( const std::string& name )
    {
        return false;
    }
    
}
