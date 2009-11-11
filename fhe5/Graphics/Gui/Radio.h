#ifndef FHE_GRAPHICS_RADIO_H
#define FHE_GRAPHICS_RADIO_H

#include <Graphics/Prims/SceneNode2.h>

namespace fhe
{
    namespace Graphics
    {
        class Radio : public SceneNode2
        {
            private:
                EntityPtr m_circle, m_circleEdge, m_text, m_highlight;
                
            public:
                FHE_FUNC_DECL(on_attach);
                FHE_FUNC_DECL(set_selected);
                FHE_FUNC_DECL(msg_mouseButtonDown);
                FHE_FUNC_DECL(msg_mouseButtonUp);
        };
    }
}

#endif
