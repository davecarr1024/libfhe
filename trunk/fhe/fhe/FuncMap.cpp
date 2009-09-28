#include "FuncMap.h"
#include "Var.h"
#include "VarMap.h"

#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Vec3.h"
#include "math/Quat.h"

#include <cassert>

namespace fhe
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
        m_funcs.clear();
    }
    
    void FuncMap::removeFunc( const std::string& name )
    {
        m_funcs.erase(name);
    }
    
    void FuncMap::addFunc( const std::string& name, AbstractFunc* func )
    {
        assert(func);
        removeFunc(name);
        m_funcs[name] = func;
    }
    
    bool FuncMap::hasFuncName( const std::string& name )
    {
        return m_funcs.find(name) != m_funcs.end();
    }
}
