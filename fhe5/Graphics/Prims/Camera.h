#ifndef FHE_GRAPHICS_CAMERA_H
#define FHE_GRAPHICS_CAMERA_H

#include "SceneNode3.h"

namespace fhe
{
    namespace Graphics
    {
        class Camera : public SceneNode3
        {
            public:
                FHE_FUNC_DECL(msg_render3);
        };
    }
}

#endif
