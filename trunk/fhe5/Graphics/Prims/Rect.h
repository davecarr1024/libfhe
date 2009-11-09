#ifndef GRAPHICS_RECT_H
#define GRAPHICS_RECT_H

#include "SceneNode2.h"

namespace fhe
{
    namespace Graphics
    {
        class Rect : public SceneNode2
        {
            public:
                FHE_FUNC_DECL(Rect,msg_render2);
        };
    }
}

#endif
