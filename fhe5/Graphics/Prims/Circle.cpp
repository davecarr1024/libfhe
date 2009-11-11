#include "Circle.h"
#include <fhe/math/fheMath.h>
#include <fhe/math/Vec2.h>
#include <SDL/SDL_opengl.h>

namespace fhe
{
    namespace Graphics
    {
        FHE_ASPECT(Circle,SceneNode2);
        
        FHE_FUNC_IMPL(Circle,msg_render2)
        {
            int slices = getEntity()->getVar<int>("slices",32);
            
            if ( getEntity()->getVar<bool>("filled",true) )
            {
                glBegin(GL_TRIANGLE_FAN);
                glTexCoord2f(0.5,0.5);
                glVertex2f(0,0);
            }
            else
            {
                glBegin(GL_LINE_LOOP);
            }
            
            float th, x, y;
            
            for ( int i = 0; i < slices+1; ++i )
            {
                th = Math::TWO_PI * float(i) / float(slices);
                x = Math::cos(th);
                y = Math::sin(th);
                glTexCoord2f(0.5 + 0.5 * x, 0.5 + 0.5 * y );
                glVertex2f(x*0.5,y*0.5);
            }
            
            glEnd();
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Circle,collTest)
        {
            return Var::build<bool>(arg.get<Vec2>().length() < 0.5);
        }
    }
}

