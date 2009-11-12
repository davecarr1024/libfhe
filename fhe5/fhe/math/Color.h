#ifndef COLOR_H
#define COLOR_H

#include "fheMath.h"

namespace fhe
{
    
    class Color
    {
        public:
            float r, g, b, a;
            
            Color( float _r = 1, float _g = 1, float _b = 1, float _a = 1 );
            
            Color lighten();
            
            Color darken();
            
            Color normalize();
            
            Color operator*(float f);
    };
    
}

#endif
