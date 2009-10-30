#include "FuncMap.h"

namespace gge
{
    
    FuncMap::FuncMap()
    {
    }
    
    FuncMap::~FuncMap()
    {
        clearFuncs();
    }
    
    void FuncMap::clearFuncs()
    {
        for ( std::map<std::string,AbstractFunc*>::iterator i = m_funcs.begin(); i != m_funcs.end(); ++i )
        {
            delete i->second;
        }
        m_funcs.clear();
    }
    
    bool FuncMap::hasFuncName( const std::string& name )
    {
        return m_funcs.find(name) != m_funcs.end();
    }
    
    void FuncMap::addFunc( const std::string& name, AbstractFunc* func )
    {
        if ( hasFuncName( name ) )
        {
            delete m_funcs[name];
        }
        m_funcs[name] = func;
    }
    
}
