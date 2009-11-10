#include "PyFunc.h"
#include "PyEnv.h"

namespace fhe
{
    namespace Python
    {
        
        PyFunc::PyFunc( const std::string& name, boost::python::object self, boost::python::object func ) :
            m_name(name),
            m_self(self),
            m_func(func)
        {
        }
        
        std::string PyFunc::getName()
        {
            return m_name;
        }
        
        Var PyFunc::call( const Var& arg )
        {
            return PyEnv::instance().convertToVar(m_func(m_self,PyEnv::instance().convertFromVar(arg)));
        }
        
    }
}
