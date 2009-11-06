#include "VarMap.h"

namespace fhe
{
    
    bool VarMap::hasVarName( const std::string& name )
    {
        return m_vars.find(name) != m_vars.end();
    }
    
    Var VarMap::getRawVar( const std::string& name )
    {
        return hasVarName(name) ? m_vars[name] : Var();
    }
    
    void VarMap::setRawVar( const std::string& name, const Var& val )
    {
        m_vars[name] = val;
    }
    
}
