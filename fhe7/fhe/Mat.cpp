#include <fhe/Mat.h>
#include <fhe/Vec.h>
#include <fhe/Rot.h>
#include <fhe/Util.h>
#include <sstream>

namespace fhe
{
    
    const Mat2 Mat2::ZERO( 0, 0, 0,
                           0, 0, 0,
                           0, 0, 0 );
                           
    const Mat2 Mat2::IDENTITY( 1, 0, 0,
                               0, 1, 0, 
                               0, 0, 1 );
    
    Mat2::Mat()
    {
        *this = IDENTITY;
    }
    
    Mat2::Mat( double d0, double d1, double d2, 
               double d3, double d4, double d5, 
               double d6, double d7, double d8 )
    {
        m_d[0] = d0;
        m_d[1] = d1;
        m_d[2] = d2;
        m_d[3] = d3;
        m_d[4] = d4;
        m_d[5] = d5;
        m_d[6] = d6;
        m_d[7] = d7;
        m_d[8] = d8;
    }
    
    Mat2::Mat( const Mat& m )
    {
        for ( size_t i = 0; i < 9; ++i )
        {
            m_d[i] = m.m_d[i];
        }
    }
    
    Mat2& Mat2::operator=( const Mat& m )
    {
        for ( size_t i = 0; i < 9; ++i )
        {
            m_d[i] = m.m_d[i];
        }
        return *this;
    }
    
    Mat2 Mat2::translation( const Vec2& v )
    {
        return Mat2( 1, 0, v.x,
                     0, 1, v.y,
                     0, 0, 1 );
    }
    
    Mat2 Mat2::rotation( const Rot2& r )
    {
        double sa = Math::sin( -r.radians() );
        double ca = Math::cos( -r.radians() );
        return Mat2( ca, sa, 0,
                     -sa, ca, 0,
                     0, 0, 1 );
    }
    
    Mat2 Mat2::scale( const Vec2& v )
    {
        return Mat2( v.x, 0, 0,
                     0, v.y, 0,
                     0, 0, 1 );
    }
    
    double Mat2::operator()( size_t i, size_t j ) const
    {
        FHE_ASSERT( i < 3 && j < 3 );
        return m_d[ i * 3 + j ];
    }
    
    double& Mat2::operator()( size_t i, size_t j )
    {
        FHE_ASSERT( i < 3 && j < 3 );
        return m_d[ i * 3 + j ];
    }
    
    Mat2 Mat2::operator*( const Mat2& m ) const
    {
        Mat2 result = ZERO;
        size_t i, j, k;
        for ( i = 0; i < 3; ++i )
        {
            for ( j = 0; j < 3; ++j )
            {
                for ( k = 0; k < 3; ++k )
                {
                    result(i,j) += (*this)(i,k) * m(k,j);
                }
            }
        }
        return result;
    }
    
    Vec2 Mat2::operator*( const Vec2& v ) const
    {
        return Vec2( m_d[0] * v.x + m_d[1] * v.y + m_d[2],
                     m_d[3] * v.x + m_d[4] * v.y + m_d[5] );
    }
    
    
    double Mat2::det() const
    {
        return m_d[0] * (m_d[4] * m_d[8] - m_d[7] * m_d[5]) -
               m_d[1] * (m_d[3] * m_d[8] - m_d[6] * m_d[5]) +
               m_d[2] * (m_d[3] * m_d[7] - m_d[6] * m_d[4]);
    }
    
    Mat2 Mat2::inverse() const
    {
        double d = det();
        FHE_ASSERT( !Math::equal( d, 0 ) );
        return Mat( (m_d[4] * m_d[8] - m_d[5] * m_d[7]) / d,
                    -(m_d[1] * m_d[8] - m_d[7] * m_d[2]) / d,
                     (m_d[1] * m_d[5] - m_d[4] * m_d[2]) / d,
                    -(m_d[3] * m_d[8] - m_d[5] * m_d[6]) / d,
                     (m_d[0] * m_d[8] - m_d[6] * m_d[2]) / d,
                    -(m_d[0] * m_d[5] - m_d[3] * m_d[2]) / d,
                     (m_d[3] * m_d[7] - m_d[6] * m_d[4]) / d,
                    -(m_d[0] * m_d[7] - m_d[6] * m_d[1]) / d,
                     (m_d[0] * m_d[4] - m_d[1] * m_d[3]) / d);
    }
    
    bool Mat2::equals( const Mat& m, double eps ) const
    {
        for ( size_t i = 0; i < 9; ++i )
        {
            if ( !Math::equal( m_d[i], m.m_d[i], eps ) )
            {
                return false;
            }
        }
        return true;
    }
    
    bool Mat2::operator==( const Mat& m ) const
    {
        return equals( m );
    }
    
    std::string Mat2::toString() const
    {
        std::ostringstream os;
        os << "Mat2(";
        for ( size_t i = 0; i < 9; ++i )
        {
            os << m_d[i];
            if ( i != 8 )
            {
                os << ",";
            }
        }
        os << ")";
        return os.str();
    }
    
    std::ostream& operator<<( std::ostream& os, const Mat2& m )
    {
        return os << m.toString();
    }
    
    double Mat2::pyGet( boost::python::tuple key ) const
    {
        FHE_ASSERT( boost::python::len( key ) == 2 );
        size_t i = boost::python::extract<size_t>( key[0] );
        size_t j = boost::python::extract<size_t>( key[1] );
        FHE_ASSERT( i < 3 && j < 3 );
        return (*this)(i,j);
    }
    
    void Mat2::pySet( boost::python::tuple key, double d )
    {
        FHE_ASSERT( boost::python::len( key ) == 2 );
        size_t i = boost::python::extract<size_t>( key[0] );
        size_t j = boost::python::extract<size_t>( key[1] );
        FHE_ASSERT( i < 3 && j < 3 );
        (*this)(i,j) = d;
    }
    
    boost::python::object Mat2::defineClass()
    {
        boost::python::scope c = boost::python::class_<Mat2>( "Mat2", boost::python::init<>() )
            .def( boost::python::init<double,double,double,double,double,double,double,double,double>() )
            .def( "translation", &Mat::translation )
            .staticmethod( "translation" )
            .def( "rotation", &Mat::rotation )
            .staticmethod( "rotation" )
            .def( "scale", &Mat::scale )
            .staticmethod( "scale" )
            .def( "__getitem__", &Mat::pyGet )
            .def( "__setitem__", &Mat::pySet )
            .def( "__repr__", &Mat::toString )
            .def( "__er__", &Mat::operator== )
            .def( boost::python::self * boost::python::other<Mat2>() )
            .def( boost::python::self * boost::python::other<Vec2>() )
            .def( "det", &Mat::det )
            .def( "inverse", &Mat::inverse )
        ;
        
        c.attr( "ZERO" ) = ZERO;
        c.attr( "IDENTITY" ) = IDENTITY;
        
        return c;
    }
    
    const Mat3 Mat3::ZERO( 0, 0, 0, 0, 
                           0, 0, 0, 0, 
                           0, 0, 0, 0, 
                           0, 0, 0, 0 );
                           
    const Mat3 Mat3::IDENTITY( 1, 0, 0, 0,
                               0, 1, 0, 0, 
                               0, 0, 1, 0, 
                               0, 0, 0, 1 );
    
    Mat3::Mat()
    {
        *this = IDENTITY;
    }
    
    Mat3::Mat( double d0, double d1, double d2, double d3, 
               double d4, double d5, double d6, double d7, 
               double d8, double d9, double d10, double d11, 
               double d12, double d13, double d14, double d15 )
    {
        m_d[0] = d0;
        m_d[1] = d1;
        m_d[2] = d2;
        m_d[3] = d3;
        m_d[4] = d4;
        m_d[5] = d5;
        m_d[6] = d6;
        m_d[7] = d7;
        m_d[8] = d8;
        m_d[9] = d9;
        m_d[10] = d10;
        m_d[11] = d11;
        m_d[12] = d12;
        m_d[13] = d13;
        m_d[14] = d14;
        m_d[15] = d15;
    }
    
    Mat3::Mat( const Mat& m )
    {
        for ( size_t i = 0; i < 16; ++i )
        {
            m_d[i] = m.m_d[i];
        }
    }
    
    Mat3& Mat3::operator=( const Mat& m )
    {
        for ( size_t i = 0; i < 16; ++i )
        {
            m_d[i] = m.m_d[i];
        }
        return *this;
    }
    
    Mat3 Mat3::translation( const Vec3& v )
    {
        return Mat3( 1, 0, 0, v.x,
                     0, 1, 0, v.y,
                     0, 0, 1, v.z,
                     0, 0, 0, 1 );
    }
    
    Mat3 Mat3::rotation( const Rot3& r )
    {
        double xx = r.x * r.x,
              xy = r.x * r.y,
              xz = r.x * r.z,
              xw = r.x * r.w,
              yy = r.y * r.y,
              yz = r.y * r.z,
              yw = r.y * r.w,
              zz = r.z * r.z,
              zw = r.z * r.w;
        return Mat(1 - 2 * (yy + zz),  //0
                    2 * (xy - zw),      //1
                    2 * (xz + yw),      //2
                    0,                  //3
                    2 * (xy + zw),      //4
                    1 - 2 * (xx + zz),  //5
                    2 * (yz - xw),      //6
                    0,                  //7
                    2 * (xz - yw),      //8
                    2 * (yz + xw),      //9
                    1 - 2 * (xx + yy),  //10
                    0,                  //11
                    0,0,0,1);
    }
    
    Mat3 Mat3::scale( const Vec3& v )
    {
        return Mat3( v.x, 0, 0, 0,
                     0, v.y, 0, 0,
                     0, 0, v.z, 0,
                     0, 0, 0, 1 );
    }
    
    double Mat3::operator()( size_t i, size_t j ) const
    {
        FHE_ASSERT( i < 4 && j < 4 );
        return m_d[ i * 4 + j ];
    }
    
    double& Mat3::operator()( size_t i, size_t j )
    {
        FHE_ASSERT( i < 4 && j < 4 );
        return m_d[ i * 4 + j ];
    }
    
    Mat3 Mat3::operator*( const Mat& m ) const
    {
        Mat result = ZERO;
        size_t i, j, k;
        for ( i = 0; i < 4; ++i )
        {
            for ( j = 0; j < 4; ++j )
            {
                for ( k = 0; k < 4; ++k )
                {
                    result(i,j) += (*this)(i,k) * m(k,j);
                }
            }
        }
        return result;
    }
    
    Vec3 Mat3::operator*( const Vec3& v ) const
    {
        return Vec3( m_d[0] * v.x + m_d[1] * v.y + m_d[2] * v.z + m_d[3],
                     m_d[4] * v.x + m_d[5] * v.y + m_d[6] * v.z + m_d[7],
                     m_d[8] * v.x + m_d[9] * v.y + m_d[10] * v.z + m_d[11] );
    }
    
    Vec3 Mat3::getTranslation() const
    {
        return Vec3( m_d[3], m_d[7], m_d[11] );
    }
    
    Rot3 Mat3::getRotation() const
    {
        double t = 1 + m_d[0] + m_d[5] + m_d[10];
        if (t > Math::EPS)
        {
            double s = Math::sqrt(t) * 2.0;
            return Rot3(0.25 * s,
                        (m_d[9] - m_d[6]) / s,
                        (m_d[2] - m_d[8]) / s,
                        (m_d[4] - m_d[1]) / s);
        }
        else if (m_d[0] > m_d[5] && m_d[0] > m_d[10])
        {
            double s = Math::sqrt(1.0 + m_d[0] - m_d[5] - m_d[10]) * 2.0;
            return Rot3((m_d[9] - m_d[6]) / s,
                        0.25 * s,
                        (m_d[4] + m_d[1]) / s,
                        (m_d[2] + m_d[8]) / s);
        }
        else if (m_d[5] > m_d[10])
        {
            double s = Math::sqrt(1.0 + m_d[5] - m_d[0] - m_d[10]) * 2.0;
            return Rot3((m_d[2] - m_d[8]) / s,
                        (m_d[4] + m_d[1]) / s,
                        0.25 * s,
                        (m_d[2] - m_d[8]) / s);
        }
        else
        {
            double s = Math::sqrt(1.0 + m_d[10] - m_d[0] - m_d[5]) * 2.0;
            return Rot3((m_d[4] - m_d[1]) / s,
                        (m_d[2] + m_d[8]) / s,
                        (m_d[9] + m_d[6]) / s,
                        0.25 * s);
        }
    }
    
    Mat2 Mat3::submat( size_t i, size_t j ) const
    {
        Mat2 m;
        size_t si, sj, di, dj;
        for (di = 0; di < 3; ++di)
        {
            for (dj = 0; dj < 3; ++dj)
            {
                if (di >= i)
                    si = di + 1;
                else
                    si = di;
                if (dj >= j)
                    sj = dj + 1;
                else
                    sj = dj;
                m(di,dj) = (*this)(si,sj);
            }
        }
        return m;
    }
    
    double Mat3::det() const
    {
        double result = 0;
        int i = 1;
        for (size_t n = 0; n < 4; ++n)
        {
            result += m_d[n*4] * submat(n,0).det() * i;
            i *= -1;
        }
        return result;
    }
    
    Mat3 Mat3::inverse() const
    {
        double d = det();
        if (Math::abs(d) < Math::EPS )
            return Mat3::IDENTITY;
        else
        {
            Mat3 m;
            size_t i, j;
            int sign;
            for (i = 0; i < 4; ++i)
                for (j = 0; j < 4; ++j) 
                {
                    sign = 1 - ((i + j) % 2) * 2;
                    m.m_d[i + j*4] = (submat(i,j).det() * sign) / d;
                }
            return m;
        }
    }
    
    bool Mat3::equals( const Mat& m, double eps ) const
    {
        for ( size_t i = 0; i < 16; ++i )
        {
            if ( !Math::equal( m_d[i], m.m_d[i], eps ) )
            {
                return false;
            }
        }
        return true;
    }
    
    bool Mat3::operator==( const Mat& m ) const
    {
        return equals( m );
    }
    
    std::string Mat3::toString() const
    {
        std::ostringstream os;
        os << "Mat3(" << getTranslation().toString() << "," << getRotation().toString() << ")";
        return os.str();
    }
    
    std::ostream& operator<<( std::ostream& os, const Mat3& m )
    {
        return os << m.toString();
    }
    
    double Mat3::pyGet( boost::python::tuple key ) const
    {
        FHE_ASSERT( boost::python::len( key ) == 2 );
        size_t i = boost::python::extract<size_t>( key[0] );
        size_t j = boost::python::extract<size_t>( key[1] );
        FHE_ASSERT( i < 3 && j < 3 );
        return (*this)(i,j);
    }
    
    void Mat3::pySet( boost::python::tuple key, double d ) 
    {
        FHE_ASSERT( boost::python::len( key ) == 2 );
        size_t i = boost::python::extract<size_t>( key[0] );
        size_t j = boost::python::extract<size_t>( key[1] );
        FHE_ASSERT( i < 3 && j < 3 );
        (*this)(i,j) = d;
    }
    
    boost::python::object Mat3::defineClass()
    {
        boost::python::scope c = boost::python::class_<Mat3>( "Mat3", boost::python::init<>() )
/*            .def( boost::python::init<double,double,double,double,
                                      double,double,double,double,
                                      double,double,double,double,
                                      double,double,double,double>() )*/
            .def( "translation", &Mat3::translation )
            .staticmethod( "translation" )
            .def( "rotation", &Mat3::rotation )
            .staticmethod( "rotation" )
            .def( "scale", &Mat3::scale )
            .staticmethod( "scale" )
            .def( "getTranslation", &Mat3::getTranslation )
            .def( "getRotation", &Mat3::getRotation )
            .def( "submat", &Mat3::submat )
            .def( "det", &Mat3::det )
            .def( "inverse", &Mat3::inverse )
            .def( "__eq__", &Mat3::operator== )
            .def( "__getitem__", &Mat3::pyGet )
            .def( "__setitem__", &Mat3::pySet )
        ;
        
        c.attr( "ZERO" ) = ZERO;
        c.attr( "IDENTITY" ) = IDENTITY;
        
        return c;
    }
    
}
