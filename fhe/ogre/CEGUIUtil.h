#ifndef CEGUI_UTIL_H
#define CEGUI_UTIL_H

#include <CEGUI.h>
#include <fhe/math/Vec2.h>

namespace fhe
{
    
    class CEGUIUtil
    {
        public:
            static CEGUI::UVector2 Vec2ToCEGUIVec2( const Vec2& v );
            static Vec2 CEGUIVec2ToVec2( const CEGUI::UVector2& v );
    };
}

#endif
