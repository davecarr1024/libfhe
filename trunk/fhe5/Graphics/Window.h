#ifndef GRAPHICS_WINDOW_H
#define GRAPHICS_WINDOW_H

#include <fhe/Aspect.h>

#include "SDL/SDL.h"
#include "SDL/SDL_opengl.h"

namespace fhe
{
    namespace Graphics
    {
        
        class Window : public Aspect
        {
            private:
                SDL_Surface* m_screen;
                
                void set2dProjection();
                void set3dProjection();
                
            public:
                FHE_FUNC_DECL(on_attach);
                FHE_FUNC_DECL(msg_update);
        };
        
    }
}

#endif
