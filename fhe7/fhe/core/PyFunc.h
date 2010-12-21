#ifndef FHE_PYFUNC_H
#define FHE_PYFUNC_H

#include <fhe/core/Func.h>
#include <fhe/core/PyEnv.h>

namespace fhe
{
    
    class PyFunc : public IFunc
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func );

            std::string name() const;
            
            Val call( const std::vector< Val >& args );
            bool tryCall( const std::vector< Val >& args, Val& ret );
    };
    
}

#endif
