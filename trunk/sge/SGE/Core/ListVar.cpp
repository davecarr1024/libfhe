#include "ListVar.h"
#include <cassert>

namespace SGE
{
    
    ListVar::ListVar() :
        Var( LIST )
    {
    }
    
    bool ListVar::empty() const
    {
        return m_list.empty();
    }
    
    int ListVar::size() const
    {
        return m_list.size();
    }
    
    VarPtr ListVar::get( int pos )
    {
        assert( pos >= 0 && pos < size() );
        return m_list[pos];
    }
    
    void ListVar::append( VarPtr var )
    {
        m_list.push_back( var );
    }
    
    void ListVar::remove( VarPtr var )
    {
        for ( VarList::iterator i = m_list.begin(); i != m_list.end(); ++i)
        {
            if ( i->get() == var.get() )
            {
                m_list.erase( i );
                return;
            }
        }
    }
    
    bool ListVar::contains( VarPtr var ) const
    {
        for ( VarList::const_iterator i = m_list.begin(); i != m_list.end(); ++i)
        {
            if ( i->get() == var.get() )
            {
                return true;
            }
        }
        return false;
    }
    
    void ListVar::clear()
    {
        m_list.clear();
    }
}
