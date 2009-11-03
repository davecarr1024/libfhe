#ifndef CEGUIUTIL_H
#define CEGUIUTIL_H

#include <gge/math/Vec2.h>
#include <CEGUI.h>

namespace gge
{
    namespace Graphics
    {
    
        class CEGUIUtil
        {
            public:
                static CEGUI::UVector2 fromVec2( const Vec2& v );
                static Vec2 toVec2( const CEGUI::UVector2& v );
        };

    }
}

#endif
