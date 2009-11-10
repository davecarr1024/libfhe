#include "VarMap.h"

namespace fhe
{
    
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
            return i != m_vars.end() ? i->second : Var();
        }
    }
    
    void VarMap::setRawVar( const std::string& name, const Var& val )
    {
        m_vars[name] = val;
        onSetVar(name,val);
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
