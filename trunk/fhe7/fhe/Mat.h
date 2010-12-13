#ifndef FHE_MAT_H
#define FHE_MAT_H

#include <fhe/fhe_math.h>
#include <fhe/PyEnv.h>
#include <string>

namespace fhe
{
    
    template <size_t dim>
    class Vec;
    
    template <size_t dim>
    class Rot;
    
    template <size_t dim>
    class Mat;
    
    typedef Mat<2> Mat2;
    typedef Mat<3> Mat3;
    
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
            
            static Mat translation( const Vec<2>& v );
            static Mat rotation( const Rot<2>& r );
            static Mat scale( const Vec<2>& v );
            
            double operator()( size_t i, size_t j ) const;
            double& operator()( size_t i, size_t j );
            
            Mat operator*( const Mat& m ) const;
            Vec<2> operator*( const Vec<2>& v ) const;
            
            bool operator==( const Mat& m ) const;
            
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
            
            static Mat translation( const Vec<3>& v );
            static Mat rotation( const Rot<3>& r );
            static Mat scale( const Vec<3>& s );
            
            double operator()( size_t i, size_t j ) const;
            double& operator()( size_t i, size_t j );
            
            Mat operator*( const Mat& m ) const;
            Vec<3> operator*( const Vec<3>& v ) const;
            
            bool operator==( const Mat& m ) const;
            
            Vec<3> getTranslation() const;
            Rot<3> getRotation() const;
            
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
