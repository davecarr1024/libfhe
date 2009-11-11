#include "VarList.h"

namespace fhe
{
    
    int VarList::length() const
    {
        return m_vars.size();
    }
    
    Var VarList::getRawVar( int i ) const
    {
        assert(i >= 0 && i < length());
        return m_vars[i];
    }
    
    void VarList::appendRaw( const Var& var )
    {
        m_vars.push_back(var);
    }
    
    void VarList::remove( int pos )
    {
        assert(pos >= 0 && pos < length());
        int i;
        std::vector<Var>::iterator iter;
        for ( iter = m_vars.begin(), i = 0; iter != m_vars.end() && i < pos; ++iter, ++i );
        m_vars.erase(iter);
    }
    
    bool VarList::empty() const
    {
        return m_vars.empty();
    }
    
}
