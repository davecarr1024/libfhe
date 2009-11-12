#ifndef TEXTBOX_H
#define TEXTBOX_H

#include "Widget.h"

namespace fhe
{
    namespace Graphics
    {
        
        class TextBox : public Widget
        {
            private:
                EntityPtr m_bg, m_edge, m_text;
                
                void updateFocus();
                
            public:
                FHE_FUNC_DECL(on_attach);
                FHE_FUNC_DECL(msg_mouseButtonDown);
                FHE_FUNC_DECL(msg_keyDown);
                FHE_FUNC_DECL(set_text);
        };
        
    }
}

#endif
