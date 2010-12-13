#include <fhe/Val.h>
#include <fhe/PyEnv.h>

namespace fhe
{
    
    Val::Val() :
        m_data( 0 )
    {
    }
    
    class ValExtractor
    {
        private:
            boost::python::object* m_obj;
            Val* m_val;
            
        public:
            ValExtractor( boost::python::object* obj, Val* val ) :
                m_obj( obj ),
                m_val( val )
            {
            }
            
            template <class T>
            void operator()( T )
            {
                if ( m_val->empty() )
                {
                    try
                    {
                        m_val->set<T>( boost::python::extract<T>( *m_obj ) );
                    }
                    catch ( boost::python::error_already_set )
                    {
                    }
                }
            }
    };

    Val::Val( boost::python::object o ) :
        m_data( 0 )
    {
        boost::mpl::for_each<python_convertable_types>( ValExtractor( &o, this ) );
        FHE_ASSERT_MSG( !empty(), "unable to build val from python type %s", PyEnv::instance().getType( o ).c_str() );
        printf( "set %s %s\n", PyEnv::instance().getType( o ).c_str(), type().c_str() );
    }
    
    Val::Val( const Val& v ) :
        m_data( v.m_data ? v.m_data->clone() : 0 )
    {
    }
    
    Val& Val::operator=( const Val& v )
    {
        clear();
        m_data = v.m_data ? v.m_data->clone() : 0;
        return *this;
    }
    
    Val::~Val()
    {
        clear();
    }
    
    bool Val::empty() const
    {
        return !m_data;
    }
    
    void Val::clear()
    {
        if ( m_data )
        {
            delete m_data;
            m_data = 0;
        }
    }
    
    std::string Val::type() const
    {
        return m_data ? m_data->type() : "null";
    }
    
}
