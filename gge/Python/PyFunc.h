#ifndef PYFUNC_H
#define PYFUNC_H

#include <gge/Func.h>
#include <boost/python.hpp>

namespace gge
{
    namespace Python
    {
        
        class PyFunc : public AbstractFunc
        {
            private:
                boost::python::object m_self, m_func;
                
                bool m_hasSelf;
            
            public:
                PyFunc( boost::python::object func );
                
                PyFunc( boost::python::object self, boost::python::object func );
                
                Var call( const Var& arg );
        };
        
    }
}

#endif
