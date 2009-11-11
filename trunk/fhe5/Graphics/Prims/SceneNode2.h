#ifndef GRAPHICS_SCENENODE2_H
#define GRAPHICS_SCENENODE2_H

#include <fhe/Aspect.h>

namespace fhe
{
    namespace Graphics
    {
        
        class SceneNode2 : public Aspect
        {
            public:
                FHE_FUNC_DECL(msg_render2);
                FHE_FUNC_DECL(unmsg_render2);
                FHE_FUNC_DECL(get_localTransform);
                FHE_FUNC_DECL(get_globalTransform);
                FHE_FUNC_DECL(get_inverseGlobalTransform);
                FHE_FUNC_DECL(msg_mouseButtonDown);
                FHE_FUNC_DECL(msg_mouseButtonUp);
                FHE_FUNC_DECL(collides);
                FHE_FUNC_DECL(globalToLocal);
                FHE_FUNC_DECL(localToGlobal);
        };
        
    }
}

#endif
