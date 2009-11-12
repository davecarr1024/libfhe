#ifndef WIDGET_H
#define WIDGET_H

#include <Graphics/Prims/SceneNode2.h>

namespace fhe
{
    namespace Graphics
    {
        class Widget : public SceneNode2
        {
            public:
                FHE_FUNC_DECL(on_attach);
        };
    }
}

#endif
