#include <fhe/fhe_math.h>
#include <cmath>

namespace fhe
{
    
    const double Math::PI = 3.1415926535897931;
    const double Math::EPS = 1e-5;
    
    double Math::abs( double d )
    {
        return ::fabs( d );
    }
    
    bool Math::equal( double d1, double d2, double eps )
    {
        return abs( d1 - d2 ) < eps;
    }
    
    double Math::sqrt( double d )
    {
        return ::sqrt( d );
    }
    
    double Math::degrees( double radians )
    {
        return radians * 180.0 / PI;
    }
    
    double Math::radians( double degrees )
    {
        return degrees * PI / 180.0;
    }
    
    double Math::atan2( double y, double x )
    {
        return ::atan2( y, x );
    }
    
    double Math::cos( double d )
    {
        return ::cos( d );
    }
    
    double Math::sin( double d )
    {
        return ::sin( d );
    }
    
    double Math::acos( double d )
    {
        return ::acos( d );
    }
    
    double Math::asin( double d )
    {
        return ::asin( d );
    }
    
}
