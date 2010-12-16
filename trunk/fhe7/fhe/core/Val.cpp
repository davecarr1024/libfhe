#include <fhe/core/Val.h>
#include <fhe/core/PyEnv.h>

namespace fhe
{

    Val::Val() :
        m_data( 0 )
    {
    }
    
    Val::Val( boost::python::object o ) :
        m_data( 0 )
    {
        if ( o != boost::python::object() )
        {
            std::string type = PyEnv::instance().getType( o );
            if ( type == "bool" )
            {
                set<bool>( boost::python::extract<bool>( o ) );
            }
            else if ( type == "int" )
            {
                set<int>( boost::python::extract<int>( o ) );
            }
            else if ( type == "float" )
            {
                set<double>( boost::python::extract<double>( o ) );
            }
            else if ( type == "str" )
            {
                set<std::string>( boost::python::extract<std::string>( o ) );
            }
            else if ( type == "Vec2" )
            {
                set<Vec2>( boost::python::extract<Vec2>( o ) );
            }
            else if ( type == "Vec3" )
            {
                set<Vec3>( boost::python::extract<Vec3>( o ) );
            }
            else if ( type == "Rot2" )
            {
                set<Rot2>( boost::python::extract<Rot2>( o ) );
            }
            else if ( type == "Rot3" )
            {
                set<Rot3>( boost::python::extract<Rot3>( o ) );
            }
            else if ( type == "Mat2" )
            {
                set<Mat2>( boost::python::extract<Mat2>( o ) );
            }
            else if ( type == "Mat3" )
            {
                set<Mat3>( boost::python::extract<Mat3>( o ) );
            }
            else
            {
                FHE_ERROR( "unable to convert unknown python type %s to Val", type.c_str() );
            }
        }
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
    
    boost::python::object Val::toPy() const
    {
        return m_data ? m_data->toPy() : boost::python::object();
    }
}
