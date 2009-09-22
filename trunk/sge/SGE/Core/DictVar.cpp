#include "DictVar.h"

#include <cassert>

namespace SGE
{
    
    DictVar::DictVar() :
        Var( DICT )
    {
    }
    
    bool DictVar::contains( const std::string& key )
    {
        return m_dict.find(key) != m_dict.end();
    }
    
    VarPtr DictVar::get( const std::string& key )
    {
        return get(key, 0);
    }
    
    VarPtr DictVar::get( const std::string& key, VarPtr def )
    {
        VarMap::iterator i = m_dict.find(key);
        if (i == m_dict.end())
            return def;
        else
            return i->second;
    }
    
    void DictVar::set( const std::string& key, VarPtr val )
    {
        m_dict[key] = val;
    }
    
    VarPtr DictVar::setDefault( const std::string& key, VarPtr val )
    {
        if (!contains(key))
            set(key,val);
        return get(key);
    }
    
    bool DictVar::empty() const
    {
        return m_dict.empty();
    }
    
    void DictVar::clear()
    {
        m_dict.clear();
    }
    
    void DictVar::remove( const std::string& key )
    {
        m_dict.erase(key);
    }
    
    int DictVar::size() const
    {
        return m_dict.size();
    }
    
    std::pair<std::string, VarPtr> DictVar::get( int pos )
    {
        assert( pos >= 0 && pos < size() );
        int c = 0;
        VarMap::iterator i;
        for ( i = m_dict.begin(); c < pos; ++i, ++c );
        return *i;
    }
}
