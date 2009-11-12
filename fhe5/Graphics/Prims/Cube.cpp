#include "Cube.h"
#include <SDL/SDL_opengl.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(Cube,SceneNode3);
        
        FHE_FUNC_IMPL(Cube,msg_render3)
        {
            bool center = getEntity()->getVar<bool>("center",true);
            
            if ( center )
            {
                glTranslatef(-0.5,-0.5,-0.5);
            }
            
            glBegin(GL_QUADS);

            //top
            glTexCoord2f(0,0);
            glVertex3f(1,1,0);
            glTexCoord2f(0,1);
            glVertex3f(0,1,0);
            glTexCoord2f(1,1);
            glVertex3f(0,1,1);
            glTexCoord2f(1,0);
            glVertex3f(1,1,1);
                            
            //bottom          
            glTexCoord2f(0,0);
            glVertex3f(1,0,1);
            glTexCoord2f(0,1);
            glVertex3f(0,0,1);
            glTexCoord2f(1,1);
            glVertex3f(0,0,0);
            glTexCoord2f(1,0);
            glVertex3f(1,0,0);
                            
            //front           
            glTexCoord2f(1,0);
            glVertex3f(1,1,1);
            glTexCoord2f(0,0);
            glVertex3f(0,1,1);
            glTexCoord2f(0,1);
            glVertex3f(0,0,1);
            glTexCoord2f(1,1);
            glVertex3f(1,0,1);
                            
            //back            
            glTexCoord2f(0,0);
            glVertex3f(1,0,0);
            glTexCoord2f(0,1);
            glVertex3f(0,0,0);
            glTexCoord2f(1,1);
            glVertex3f(0,1,0);
            glTexCoord2f(1,0);
            glVertex3f(1,1,0);

            //right
            glTexCoord2f(0,0);
            glVertex3f(0,1,1);
            glTexCoord2f(0,1);
            glVertex3f(0,1,0);
            glTexCoord2f(1,1);
            glVertex3f(0,0,0);
            glTexCoord2f(1,0);
            glVertex3f(0,0,1);

            //left
            glTexCoord2f(0,0);
            glVertex3f(1,1,0);
            glTexCoord2f(0,1);
            glVertex3f(1,1,1);
            glTexCoord2f(1,1);
            glVertex3f(1,0,1);
            glTexCoord2f(1,0);
            glVertex3f(1,0,0);

            glEnd();

            if ( center )
            {
                glTranslatef(0.5,0.5,0.5);
            }
            
            return Var();
        }
        
    }
}
