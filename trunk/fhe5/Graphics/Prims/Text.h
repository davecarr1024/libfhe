#ifndef GRAPHICS_TEXT_H
#define GRAPHICS_TEXT_H

#include "SceneNode2.h"

namespace fhe
{
    namespace Graphics
    {
        class Text : public SceneNode2
        {
            public:
                FHE_FUNC_DECL(Text,msg_render2);
        };
    }
}

#endif
