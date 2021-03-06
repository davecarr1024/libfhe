#include <fhe/core/Val.h>
#include <fhe/core/Mat.h>
#include <fhe/core/Rot.h>
#include <fhe/core/Vec.h>

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
            else if ( type == "Vec2d" )
            {
                set<Vec2d>( boost::python::extract<Vec2d>( o ) );
            }
            else if ( type == "Vec3d" )
            {
                set<Vec3d>( boost::python::extract<Vec3d>( o ) );
            }
            else if ( type == "Vec2i" )
            {
                set<Vec2i>( boost::python::extract<Vec2i>( o ) );
            }
            else if ( type == "Vec3i" )
            {
                set<Vec3i>( boost::python::extract<Vec3i>( o ) );
            }
            else if ( type == "Rot2d" )
            {
                set<Rot2d>( boost::python::extract<Rot2d>( o ) );
            }
            else if ( type == "Rot3d" )
            {
                set<Rot3d>( boost::python::extract<Rot3d>( o ) );
            }
            else if ( type == "Rot2i" )
            {
                set<Rot2i>( boost::python::extract<Rot2i>( o ) );
            }
            else if ( type == "Rot3i" )
            {
                set<Rot3i>( boost::python::extract<Rot3i>( o ) );
            }
            else if ( type == "Mat2d" )
            {
                set<Mat2d>( boost::python::extract<Mat2d>( o ) );
            }
            else if ( type == "Mat3d" )
            {
                set<Mat3d>( boost::python::extract<Mat3d>( o ) );
            }
            else if ( type == "Mat2i" )
            {
                set<Mat2i>( boost::python::extract<Mat2i>( o ) );
            }
            else if ( type == "Mat3i" )
            {
                set<Mat3i>( boost::python::extract<Mat3i>( o ) );
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
