#ifndef FHE_GRAPHICS_FRAME_H
#define FHE_GRAPHICS_FRAME_H

#include "Widget.h"
#include <fhe/math/Vec2.h>

namespace fhe
{
    namespace Graphics
    {
        
        class Frame : public Widget
        {
            private:
                EntityPtr m_bg, m_titleBar, m_titleText;
                
            public:
                FHE_FUNC_DECL(on_attach);
                FHE_FUNC_DECL(msg_mouseMotion);
                FHE_FUNC_DECL(msg_mouseButtonDown);
                FHE_FUNC_DECL(msg_mouseButtonUp);
        };
        
    }
}

#endif
