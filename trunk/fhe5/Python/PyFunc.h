#ifndef PYFUNC_H
#define PYFUNC_H

#include <fhe/Func.h>
#include <boost/python.hpp>

namespace fhe
{
    namespace Python
    {
        class PyFunc : public AbstractFunc
        {
            private:
                std::string m_name;
                
                boost::python::object m_self, m_func;
            
            public:
                PyFunc( const std::string& name, boost::python::object self, boost::python::object func );
                
                std::string getName();
                
                Var call( const Var& arg );
        };
    }
}

#endif
