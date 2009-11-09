#include "VarMap.h"

namespace fhe
{
    
    bool VarMap::hasVarName( const std::string& name ) const
    {
        return m_vars.find(name) != m_vars.end();
    }
    
    Var VarMap::getRawVar( const std::string& name ) const
    {
        std::map<std::string,Var>::const_iterator i = m_vars.find(name);
        return i != m_vars.end() ? i->second : Var();
    }
    
    void VarMap::setRawVar( const std::string& name, const Var& val )
    {
        m_vars[name] = val;
    }
    
    std::vector<std::string> VarMap::getVarNames() const
    {
        std::vector<std::string> names;
        for ( std::map<std::string,Var>::const_iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            names.push_back(i->first);
        }
        return names;
    }
    
}
