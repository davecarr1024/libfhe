#include "Color.h"

namespace fhe
{
    
    Color::Color( float _r, float _g, float _b, float _a ) :
        r(_r),
        g(_g),
        b(_b),
        a(_a)
    {
    }
    
    Color Color::lighten()
    {
        return (*this * 2).normalize();
    }
    
    Color Color::darken()
    {
        return (*this * 0.5).normalize();
    }
    
    Color Color::normalize()
    {
        return Color(Math::clamp(r,0,1),Math::clamp(g,0,1),Math::clamp(b,0,1),Math::clamp(a,0,1));
    }
    
    Color Color::operator*(float f)
    {
        return Color(r*f,g*f,b*f,a);
    }
    
}
