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
                FHE_FUNC_DECL(SceneNode2,msg_render2);
                FHE_FUNC_DECL(SceneNode2,unmsg_render2);
        };
        
    }
}

#endif
