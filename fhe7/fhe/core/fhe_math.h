#ifndef FHE_MATH_H
#define FHE_MATH_H

#include <string>

namespace fhe
{
    
    template <size_t dim, typename T>
    class Vec;
    
    typedef Vec<2,double> Vec2d;
    typedef Vec<3,double> Vec3d;
    typedef Vec<2,int> Vec2i;
    typedef Vec<3,int> Vec3i;
    
    template <size_t dim>
    class Rot;
    
    typedef Rot<2> Rot2;
    typedef Rot<3> Rot3;
    
    template <size_t dim>
    class Mat;
    
    typedef Mat<2> Mat2;
    typedef Mat<3> Mat3;
    
    class Math
    {
        public:
            static const double PI;
            static const double EPS;
            
            static double abs( double d );
            static bool equal( double d1, double d2, double eps = EPS );
            static double sqrt( double d );
            static double degrees( double radians );
            static double radians( double degrees );
            static double atan2( double y, double x );
            static double cos( double d );
            static double sin( double d );
            static double acos( double d );
            static double asin( double d );
    };
    
}

#endif
