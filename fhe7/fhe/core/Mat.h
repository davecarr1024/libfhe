#ifndef FHE_MAT_H
#define FHE_MAT_H

#include <fhe/core/fhe_math.h>
#include <fhe/core/PyEnv.h>
#include <string>

namespace fhe
{
    
    template <>
    class Mat<2>
    {
        private:
            double m_d[9];
            
        public:
            Mat();
            Mat( double d0, double d1, double d2, 
                 double d3, double d4, double d5, 
                 double d6, double d7, double d8 );
            Mat( const Mat& m );
            Mat& operator=( const Mat& m );
            
            template <typename T>
            static Mat translation( const Vec<2,T>& v )
            {
                return Mat( 1, 0, v.x,
                            0, 1, v.y,
                            0, 0, 1 );
            }
            
            static Mat rotation( const Rot2& r );
            
            template <typename T>
            static Mat scale( const Vec<2,T>& v )
            {
                return Mat( v.x, 0, 0,
                            0, v.y, 0,
                            0, 0, 1 );
            }
            
            double operator()( size_t i, size_t j ) const;
            double& operator()( size_t i, size_t j );
            
            Mat operator*( const Mat& m ) const;
            
            template <typename T>
            Vec<2,T> operator*( const Vec<2,T>& v ) const
            {
                return Vec<2,T>( m_d[0] * v.x + m_d[1] * v.y + m_d[2],
                                m_d[3] * v.x + m_d[4] * v.y + m_d[5] );
            }
            
            bool operator==( const Mat& m ) const;
            
            template <typename T>
            Vec<2,T> getTranslation() const
            {
                return Vec<2,T>( (*this)(0,2), (*this)(1,2) );
            }
            
            Rot2 getRotation() const;
            double det() const;
            Mat inverse() const;
            bool equals( const Mat& m, double eps = Math::EPS ) const;
            std::string toString() const;
            
            double pyGet( boost::python::tuple key ) const;
            void pySet( boost::python::tuple key, double d );
            static boost::python::object defineClass();
            
            const static Mat ZERO;
            const static Mat IDENTITY;
    };
    
    std::ostream& operator<<( std::ostream& os, const Mat2& m );
    
    template <>
    class Mat<3>
    {
        private:
            double m_d[16];
            
        public:
            Mat();
            Mat( double d0, double d1, double d2, double d3, 
                 double d4, double d5, double d6, double d7, 
                 double d8, double d9, double d10, double d11, 
                 double d12, double d13, double d14, double d15 );
            Mat( const Mat& m );
            Mat& operator=( const Mat& m );
            
            template <typename T>
            static Mat translation( const Vec<3,T>& v )
            {
                return Mat( 1, 0, 0, v.x,
                            0, 1, 0, v.y,
                            0, 0, 1, v.z,
                            0, 0, 0, 1 );
            }
            
            static Mat rotation( const Rot3& r );
            
            template <typename T>
            static Mat scale( const Vec<3,T>& v )
            {
                return Mat( v.x, 0, 0, 0,
                            0, v.y, 0, 0,
                            0, 0, v.z, 0,
                            0, 0, 0, 1 );
            }
            
            double operator()( size_t i, size_t j ) const;
            double& operator()( size_t i, size_t j );
            
            Mat operator*( const Mat& m ) const;
            
            template <typename T>
            Vec<3,T> operator*( const Vec<3,T>& v ) const
            {
                return Vec<3,T>( m_d[0] * v.x + m_d[1] * v.y + m_d[2] * v.z + m_d[3],
                                 m_d[4] * v.x + m_d[5] * v.y + m_d[6] * v.z + m_d[7],
                                 m_d[8] * v.x + m_d[9] * v.y + m_d[10] * v.z + m_d[11] );
            }
            
            bool operator==( const Mat& m ) const;
            
            template <typename T>
            Vec<3,T> getTranslation() const
            {
                return Vec<3,T>( m_d[3], m_d[7], m_d[11] );
            }
            
            Rot3 getRotation() const;
            
            Mat2 submat( size_t i, size_t j ) const;
            double det() const;
            Mat inverse() const;
            bool equals( const Mat& m, double eps = Math::EPS ) const;
            std::string toString() const;
            
            static const Mat ZERO;
            static const Mat IDENTITY;
            
            double pyGet( boost::python::tuple key ) const;
            void pySet( boost::python::tuple key, double d );
            static boost::python::object defineClass();
    };
    
    std::ostream& operator<<( std::ostream& os, const Mat3& m );
    
}

#endif
