#ifndef FHE_IFUNC_H
#define FHE_IFUNC_H

#include <boost/shared_ptr.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/repetition.hpp>
#include <boost/preprocessor/arithmetic/sub.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>
#include <string>

#ifndef FHE_ARGS
#define FHE_ARGS 3
#endif

namespace fhe
{
    
    template <class TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, class TArg )>
    class TFunc;
    
    class IFunc
    {
        public:
            template <class TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, class TArg )>
            TFunc<TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, TArg )>* as()
            {
                return dynamic_cast<TFunc<TRet, BOOST_PP_ENUM_PARAMS( FHE_ARGS, TArg )>*>( this );
            }
            
            virtual std::string name() const = 0;
    };
    
    typedef boost::shared_ptr< IFunc > IFuncPtr;
    
}

#endif
