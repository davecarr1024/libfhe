#ifndef FHE_MATH_H
#define FHE_MATH_H

namespace fhe
{
    
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
