#include "SceneNode2.h"

namespace fhe
{
    namespace Graphics
    {
        class Circle : public SceneNode2
        {
            public:
                FHE_FUNC_DECL(msg_render2);
                FHE_FUNC_DECL(collTest);
        };
    }
}
