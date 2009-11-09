#include "Rect.h"
#include <SDL/SDL_opengl.h>

namespace fhe
{
    namespace Graphics
    {
        FHE_ASPECT(Rect,SceneNode2);
        
        FHE_FUNC_IMPL(Rect,msg_render2)
        {
            if ( getEntity()->getVar<bool>("filled",true) )
            {
                glBegin(GL_QUADS);
            }
            else
            {
                glBegin(GL_LINE_LOOP);
            }
            
            glTexCoord2f(0,0);
            glVertex2f(0,0);
            glTexCoord2f(0,1);
            glVertex2f(0,1);
            glTexCoord2f(1,1);
            glVertex2f(1,1);
            glTexCoord2f(1,0);
            glVertex2f(1,0);
            glEnd();
            
            return Var();
        }
    }
}
