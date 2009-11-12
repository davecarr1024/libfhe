#ifndef FHE_GRAPHICS_SLIDER_H
#define FHE_GRAPHICS_SLIDER_H

#include "Widget.h"

namespace fhe
{
    namespace Graphics
    {
        class Slider : public Widget
        {
            private:
                EntityPtr m_slider, m_sliderEdge, m_text, m_bg;
                
            public:
                FHE_FUNC_DECL(on_attach);
                FHE_FUNC_DECL(msg_mouseButtonDown);
                FHE_FUNC_DECL(msg_mouseButtonUp);
                FHE_FUNC_DECL(msg_mouseMotion);
                FHE_FUNC_DECL(set_value);
        };
    }
}

#endif
