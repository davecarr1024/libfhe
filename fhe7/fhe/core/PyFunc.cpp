#include <fhe/core/PyFunc.h>

namespace fhe
{
    
    PyFunc::PyFunc( boost::python::object func ) :
        m_func( func )
    {
    }
    
    std::string PyFunc::name() const
    {
        return boost::python::extract< std::string >( m_func.attr( "__name__" ) );
    }
    
    Val PyFunc::call( const std::vector< Val >& args )
    {
        switch ( args.size() )
        {
            #define ARG_iter( z, n, unused ) args[n].toPy()
            
            #define CALL_iter( z, n, unused ) \
            case n: \
                return Val( m_func( BOOST_PP_ENUM( n, ARG_iter, ~ ) ) );
                
            BOOST_PP_REPEAT( FHE_ARGS, CALL_iter, ~ )
            
            #undef CALL_iter
            #undef ARG_iter
                
            default:
                FHE_ERROR( "too many args (%d) for python call", args.size() );
                return Val();
        }
    }
    
    bool PyFunc::tryCall( const std::vector< Val >& args, Val& ret )
    {
        try
        {
            ret = call( args );
            return true;
        }
        catch ( boost::python::error_already_set )
        {
            return false;
        }
    }
    
}
