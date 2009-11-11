#include "fheMath.h"
#include <cmath>

namespace fhe
{
    
    const float Math::PI = 3.1415926535897931f;
    const float Math::TWO_PI = 6.2831853071795862f;
    const float Math::HALF_PI = 1.5707963267948966f;
    const float Math::EPSILON = 1e-5f;
    
    bool Math::equal( float f1, float f2, float eps )
    {
        return Math::fabs( f1 - f2 ) <= eps;
    }
    
    float Math::cos( float x )
    {
        return cosf(x);
    }
    
    float Math::sin( float x )
    {
        return sinf(x);
    }

    float Math::tan( float x )
    {
        return tanf(x);
    }

    float Math::asin( float x )
    {
        return asinf(x);
    }

    float Math::acos( float x )
    {
        return acosf(x);
    }
    
    float Math::atan2( float x, float y )
    {
        return atan2f(x,y);
    }
    
    float Math::sqrt( float x )
    {
        return sqrtf(x);
    }
    
    float Math::fabs( float x )
    {
        return fabsf(x);
    }
    
    float Math::fmod( float x, float y )
    {
        return fmodf(x,y);
    }
    
    float Math::degrees( float radians )
    {
        return radians * 180.0 / PI;
    }
    
    float Math::radians( float degrees )
    {
        return degrees * PI / 180.0;
    }
    
    float Math::clamp( float x, float min, float max )
    {
        if ( x < min )
        {
            return min;
        }
        else if ( x > max )
        {
            return max;
        }
        else
        {
            return x;
        }
    }
}
