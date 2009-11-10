#ifndef BUTTON_H
#define BUTTON_H

#include <Graphics/Prims/SceneNode2.h>

namespace fhe
{
    namespace Graphics
    {
        
        class Button : public SceneNode2
        {
            private:
                EntityPtr m_bg;
                
            public:
                FHE_FUNC_DECL(on_attach);
                FHE_FUNC_DECL(msg_clickUp);
        };
        
    }
}

#endif
