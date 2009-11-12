#ifndef CUBE_H
#define CUBE_H

#include "SceneNode3.h"

namespace fhe
{
    namespace Graphics
    {
        class Cube : public SceneNode3
        {
            public:
                FHE_FUNC_DECL(msg_render3);
        };
    }
}

#endif
