#ifndef FHE_TEXT_SCENE_NODE_H
#define FHE_TEXT_SCENE_NODE_H

#include <fhe/sim/SpatialNode.h>
#include <fhe/text/RenderContext.h>

namespace fhe
{
    namespace text
    {
        
        class SceneNode : public sim::SpatialNode2i
        {
            public:
                virtual void render( RenderContext* rc ) = 0;
        };
        
    }
}

#endif
