#include "CEGUIUtil.h"

namespace fhe
{
    
    CEGUI::UVector2 CEGUIUtil::Vec2ToCEGUIVec2( const Vec2& v )
    {
        return CEGUI::UVector2( CEGUI::UDim(v.x,0), CEGUI::UDim(v.y,0) );
    }
    
    Vec2 CEGUIUtil::CEGUIVec2ToVec2( const CEGUI::UVector2& v )
    {
        return Vec2( v.d_x.d_scale, v.d_y.d_scale );
    }
    
}
