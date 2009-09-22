#ifndef FHE_MATH_H
#define FHE_MATH_H

namespace fhe
{
    
    class Math
    {
        public:
            static const float PI;
            static const float HALF_PI;
            static const float TWO_PI;
            static const float EPSILON;
            
            static bool equal( float f1, float f2, float eps = Math::EPSILON );
            static float sin( float x );
            static float cos( float x );
            static float tan( float x );
            static float asin( float x );
            static float acos( float x );
            static float atan2( float x, float y );
            static float sqrt( float x );
            static float fabs( float x );
            static float fmod( float x, float y );
    };
    
}

#endif
