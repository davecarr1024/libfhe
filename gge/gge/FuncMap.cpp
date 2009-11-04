#include "FuncMap.h"
#include "Var.h"
#include "VarMap.h"

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
    
    bool FuncMap::hasFunc( const std::string& name )
    {
        return m_funcs.find(name) != m_funcs.end();
    }
    
    void FuncMap::addFunc( const std::string& name, AbstractFunc* func )
    {
        if ( hasFunc( name ) )
        {
            delete m_funcs[name];
        }
        m_funcs[name] = func;
    }
    
    Var FuncMap::call( const std::string& name, const Var& arg )
    {
        assert(hasFunc(name));
        return m_funcs[name]->call(arg);
    }
    
    AbstractFunc* FuncMap::getFunc( const std::string& name ) const
    {
        std::map<std::string,AbstractFunc*>::const_iterator i = m_funcs.find(name);
        return i != m_funcs.end() ? i->second : 0;
    }
}
