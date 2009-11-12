#ifndef BUTTON_H
#define BUTTON_H

#include "Widget.h"

namespace fhe
{
    namespace Graphics
    {
        
        class Button : public Widget
        {
            private:
                EntityPtr m_bg, m_border, m_text;
                
            public:
                FHE_FUNC_DECL(on_attach);
                FHE_FUNC_DECL(msg_clickDown);
                FHE_FUNC_DECL(msg_mouseButtonUp);
        };
        
    }
}

#endif
