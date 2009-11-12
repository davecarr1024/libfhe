#ifndef FHE_GRAPHICS_SCENENODE3_H
#define FHE_GRAPHICS_SCENENODE3_H

#include <fhe/Aspect.h>

namespace fhe
{
    namespace Graphics
    {
        class SceneNode3 : public Aspect
        {
            public:
                FHE_FUNC_DECL(msg_render3);
                FHE_FUNC_DECL(unmsg_render3);
                FHE_FUNC_DECL(get_localTransform);
                FHE_FUNC_DECL(get_globalTransform);
                FHE_FUNC_DECL(get_inverseGlobalTransform);
                FHE_FUNC_DECL(localToGlobal);
                FHE_FUNC_DECL(globalToLocal);
        };
    }
}

#endif
