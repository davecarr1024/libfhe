#ifndef FHE_MAT_H
#define FHE_MAT_H

#include <fhe/core/fhe_math.h>
#include <fhe/core/PyEnv.h>
#include <fhe/core/Util.h>

namespace fhe
{
    
    template <typename T>
    class Mat<2,T>
    {
        private:
            T m_d[9];
            
        public:
            typedef Vec<2,T> V;
            typedef Rot<2,T> R;
            
            Mat() 
            {
                *this = IDENTITY;
            }
            
            Mat( T d0, T d1, T d2, 
                 T d3, T d4, T d5, 
                 T d6, T d7, T d8 )
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
            
            Mat( const Mat& m )
            {
                for ( size_t i = 0; i < 9; ++i )
                {
                    m_d[i] = m.m_d[i];
                }
            }
            
            Mat& operator=( const Mat& m )
            {
                for ( size_t i = 0; i < 9; ++i )
                {
                    m_d[i] = m.m_d[i];
                }
                return *this;
            }
            
            static Mat translation( const V& v )
            {
                return Mat( 1, 0, v.x,
                            0, 1, v.y,
                            0, 0, 1 );
            }
            
            static Mat rotation( const R& r )
            {
                T sa = Math::sin( -r.radians() );
                T ca = Math::cos( -r.radians() );
                return Mat( ca, sa, 0,
                            -sa, ca, 0,
                            0, 0, 1 );
            }
            
            static Mat scale( const V& v )
            {
                return Mat( v.x, 0, 0,
                            0, v.y, 0,
                            0, 0, 1 );
            }
            
            T operator()( size_t i, size_t j ) const
            {
                FHE_ASSERT( i < 3 && j < 3 );
                return m_d[ i * 3 + j ];
            }
            
            T& operator()( size_t i, size_t j )
            {
                FHE_ASSERT( i < 3 && j < 3 );
                return m_d[ i * 3 + j ];
            }
            
            Mat operator*( const Mat& m ) const
            {
                Mat result = ZERO;
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
            
            V operator*( const V& v ) const
            {
                return V( m_d[0] * v.x + m_d[1] * v.y + m_d[2],
                          m_d[3] * v.x + m_d[4] * v.y + m_d[5] );
            }
            
            bool operator==( const Mat& m ) const
            {
                return equals( m );
            }
            
            V getTranslation() const
            {
                return V( (*this)(0,2), (*this)(1,2) );
            }
            
            R getRotation() const
            {
                return R::fromRadians( Math::acos( (*this)(0,0) ) ); //?
            }
            
            T det() const
            {
                return m_d[0] * (m_d[4] * m_d[8] - m_d[7] * m_d[5]) -
                    m_d[1] * (m_d[3] * m_d[8] - m_d[6] * m_d[5]) +
                    m_d[2] * (m_d[3] * m_d[7] - m_d[6] * m_d[4]);
            }
            
            Mat inverse() const
            {
                T d = det();
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
            
            bool equals( const Mat& m, T eps = Math::EPS ) const
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
            
            static std::string typeName()
            {
                return std::string( "Mat2" ) + typeid(T).name();
            }
            
            std::string toString() const
            {
                std::ostringstream os;
                os << typeName() << "(" << getTranslation() << "," << getRotation() << ")";
                return os.str();
            }
            
            T pyGet( boost::python::tuple key ) const
            {
                FHE_ASSERT( boost::python::len( key ) == 2 );
                size_t i = boost::python::extract<size_t>( key[0] );
                size_t j = boost::python::extract<size_t>( key[1] );
                FHE_ASSERT( i < 3 && j < 3 );
                return (*this)(i,j);
            }
                
            void pySet( boost::python::tuple key, T d )
            {
                FHE_ASSERT( boost::python::len( key ) == 2 );
                size_t i = boost::python::extract<size_t>( key[0] );
                size_t j = boost::python::extract<size_t>( key[1] );
                FHE_ASSERT( i < 3 && j < 3 );
                (*this)(i,j) = d;
            }
            
            static boost::python::object defineClass()
            {
                boost::python::scope c = boost::python::class_<Mat>( typeName().c_str(), boost::python::init<>() )
                    .def( boost::python::init<T,T,T,T,T,T,T,T,T>() )
                    .def( "translation", &Mat::translation )
                    .staticmethod( "translation" )
                    .def( "rotation", &Mat::rotation )
                    .staticmethod( "rotation" )
                    .def( "scale", &Mat::scale )
                    .staticmethod( "scale" )
                    .def( "getTranslation", &Mat::getTranslation )
                    .def( "getRotation", &Mat::getRotation )
                    .def( "__getitem__", &Mat::pyGet )
                    .def( "__setitem__", &Mat::pySet )
                    .def( "__repr__", &Mat::toString )
                    .def( "__er__", &Mat::operator== )
                    .def( boost::python::self * boost::python::other<Mat>() )
                    .def( boost::python::self * boost::python::other<V>() )
                    .def( "det", &Mat::det )
                    .def( "inverse", &Mat::inverse )
                ;
                
                c.attr( "ZERO" ) = ZERO;
                c.attr( "IDENTITY" ) = IDENTITY;
                
                return c;
            }
            
            const static Mat ZERO;
            const static Mat IDENTITY;
    };
    
    template <typename T>
    const Mat<2,T> Mat<2,T>::ZERO( 0, 0, 0,
                                   0, 0, 0,
                                   0, 0, 0 );
    template <typename T>
    const Mat<2,T> Mat<2,T>::IDENTITY( 1, 0, 0,
                                       0, 1, 0,
                                       0, 0, 1 );
    
    template <typename T>
    class Mat<3,T>
    {
        private:
            T m_d[16];
            
        public:
            typedef Vec<3,T> V;
            typedef Rot<3,T> R;
            
            Mat()
            {
                *this = IDENTITY;
            }
            
            Mat( T d0, T d1, T d2, T d3, 
                 T d4, T d5, T d6, T d7, 
                 T d8, T d9, T d10, T d11, 
                 T d12, T d13, T d14, T d15 )
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
            
            Mat( const Mat& m )
            {
                for ( size_t i = 0; i < 16; ++i )
                {
                    m_d[i] = m.m_d[i];
                }
            }
            
            Mat& operator=( const Mat& m )
            {
                for ( size_t i = 0; i < 16; ++i )
                {
                    m_d[i] = m.m_d[i];
                }
                return *this;
            }
            
            static Mat translation( const V& v )
            {
                return Mat( 1, 0, 0, v.x,
                            0, 1, 0, v.y,
                            0, 0, 1, v.z,
                            0, 0, 0, 1 );
            }
            
            static Mat rotation( const R& r )
            {
                T xx = r.x * r.x,
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
            
            static Mat scale( const V& v )
            {
                return Mat( v.x, 0, 0, 0,
                            0, v.y, 0, 0,
                            0, 0, v.z, 0,
                            0, 0, 0, 1 );
            }
            
            T operator()( size_t i, size_t j ) const
            {
                FHE_ASSERT( i < 4 && j < 4 );
                return m_d[ i * 4 + j ];
            }
            
            T& operator()( size_t i, size_t j )
            {
                FHE_ASSERT( i < 4 && j < 4 );
                return m_d[ i * 4 + j ];
            }
            
            Mat operator*( const Mat& m ) const
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
            
            V operator*( const V& v ) const
            {
                return V( m_d[0] * v.x + m_d[1] * v.y + m_d[2] * v.z + m_d[3],
                          m_d[4] * v.x + m_d[5] * v.y + m_d[6] * v.z + m_d[7],
                          m_d[8] * v.x + m_d[9] * v.y + m_d[10] * v.z + m_d[11] );
            }
            
            bool operator==( const Mat& m ) const
            {
                return equals( m );
            }
            
            V getTranslation() const
            {
                return V( m_d[3], m_d[7], m_d[11] );
            }
            
            R getRotation() const
            {
                T t = 1 + m_d[0] + m_d[5] + m_d[10];
                if (t > Math::EPS)
                {
                    T s = Math::sqrt(t) * 2.0;
                    return R(0.25 * s,
                                (m_d[9] - m_d[6]) / s,
                                (m_d[2] - m_d[8]) / s,
                                (m_d[4] - m_d[1]) / s);
                }
                else if (m_d[0] > m_d[5] && m_d[0] > m_d[10])
                {
                    T s = Math::sqrt(1.0 + m_d[0] - m_d[5] - m_d[10]) * 2.0;
                    return R((m_d[9] - m_d[6]) / s,
                                0.25 * s,
                                (m_d[4] + m_d[1]) / s,
                                (m_d[2] + m_d[8]) / s);
                }
                else if (m_d[5] > m_d[10])
                {
                    T s = Math::sqrt(1.0 + m_d[5] - m_d[0] - m_d[10]) * 2.0;
                    return R((m_d[2] - m_d[8]) / s,
                                (m_d[4] + m_d[1]) / s,
                                0.25 * s,
                                (m_d[2] - m_d[8]) / s);
                }
                else
                {
                    T s = Math::sqrt(1.0 + m_d[10] - m_d[0] - m_d[5]) * 2.0;
                    return R((m_d[4] - m_d[1]) / s,
                                (m_d[2] + m_d[8]) / s,
                                (m_d[9] + m_d[6]) / s,
                                0.25 * s);
                }
            }
            
            Mat<2,T> submat( size_t i, size_t j ) const
            {
                Mat<2,T> m;
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
            
            T det() const
            {
                T result = 0;
                int i = 1;
                for (size_t n = 0; n < 4; ++n)
                {
                    result += m_d[n*4] * submat(n,0).det() * i;
                    i *= -1;
                }
                return result;
            }
            
            Mat inverse() const
            {
                T d = det();
                if (Math::abs(d) < Math::EPS )
                    return IDENTITY;
                else
                {
                    Mat m;
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
            
            bool equals( const Mat& m, T eps = Math::EPS ) const
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
            
            static std::string typeName()
            {
                return std::string( "Mat3" ) + typeid(T).name();
            }
            
            std::string toString() const
            {
                std::ostringstream os;
                os << typeName() << "(" << getTranslation() << "," << getRotation() << ")";
                return os.str();
            }
            
            T pyGet( boost::python::tuple key ) const
            {
                FHE_ASSERT( boost::python::len( key ) == 2 );
                size_t i = boost::python::extract<size_t>( key[0] );
                size_t j = boost::python::extract<size_t>( key[1] );
                FHE_ASSERT( i < 3 && j < 3 );
                return (*this)(i,j);
            }
            
            void pySet( boost::python::tuple key, T d )
            {
                FHE_ASSERT( boost::python::len( key ) == 2 );
                size_t i = boost::python::extract<size_t>( key[0] );
                size_t j = boost::python::extract<size_t>( key[1] );
                FHE_ASSERT( i < 3 && j < 3 );
                (*this)(i,j) = d;
            }
            
            static boost::python::object defineClass()
            {
                boost::python::scope c = boost::python::class_<Mat>( typeName().c_str(), boost::python::init<>() )
        /*            .def( boost::python::init<double,double,double,double,
                                            double,double,double,double,
                                            double,double,double,double,
                                            double,double,double,double>() )*/
                    .def( "translation", &Mat::translation )
                    .staticmethod( "translation" )
                    .def( "rotation", &Mat::rotation )
                    .staticmethod( "rotation" )
                    .def( "scale", &Mat::scale )
                    .staticmethod( "scale" )
                    .def( "getTranslation", &Mat::getTranslation )
                    .def( "getRotation", &Mat::getRotation )
                    .def( "submat", &Mat::submat )
                    .def( "det", &Mat::det )
                    .def( "inverse", &Mat::inverse )
                    .def( "__eq__", &Mat::operator== )
                    .def( "__repr__", &Mat::toString )
                    .def( "__getitem__", &Mat::pyGet )
                    .def( "__setitem__", &Mat::pySet )
                    .def( boost::python::self * boost::python::other<Mat>() )
                    .def( boost::python::self * boost::python::other<V>() )
                ;
                
                c.attr( "ZERO" ) = ZERO;
                c.attr( "IDENTITY" ) = IDENTITY;
                
                return c;
            }
            
            static const Mat ZERO;
            static const Mat IDENTITY;
    };
    
    template <typename T>
    const Mat<3,T> Mat<3,T>::ZERO( 0, 0, 0, 0, 
                                   0, 0, 0, 0, 
                                   0, 0, 0, 0, 
                                   0, 0, 0, 0 );
                           
    template <typename T>
    const Mat<3,T> Mat<3,T>::IDENTITY( 1, 0, 0, 0,
                                       0, 1, 0, 0, 
                                       0, 0, 1, 0, 
                                       0, 0, 0, 1 );

    template <size_t dim, typename T>
    std::ostream& operator<<( std::ostream& os, const Mat<dim,T>& m )
    {
        return os << m.toString();
    }
    
}

#endif
