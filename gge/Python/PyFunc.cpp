#include "PyFunc.h"
#include "Env.h"

namespace gge
{
    namespace Python
    {
        
        PyFunc::PyFunc( boost::python::object func ) :
            m_func(func),
            m_hasSelf(false)
        {
        }
        
        PyFunc::PyFunc( boost::python::object self, boost::python::object func ) :
            m_self(self),
            m_func(func),
            m_hasSelf(true)
        {
        }
        
        Var PyFunc::call( const Var& arg )
        {
            if ( m_hasSelf )
            {
                return Env::instance().convertToVar(m_func(m_self,Env::instance().convertFromVar(arg)));
            }
            else
            {
                return Env::instance().convertToVar(m_func(Env::instance().convertFromVar(arg)));
            }
        }
        
    }
}
