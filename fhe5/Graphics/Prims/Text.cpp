#include "Text.h"
#include <Graphics/MaterialManager.h>
#include <fhe/math/Vec2.h>

namespace fhe
{
    namespace Graphics
    {
        FHE_ASPECT(Text,SceneNode2);
        
        FHE_FUNC_IMPL(Text,msg_render2)
        {
            FTFont* font = MaterialManager::instance().loadFont(getEntity()->getVar<std::string>("fontName","test.ttf"));
            int size = getEntity()->getVar<int>("fontSize",12);
            font->FaceSize(size);
                
            float m[16];
            glGetFloatv(GL_MODELVIEW_MATRIX,m);
            
            for ( int i = 0; i < 16; ++i )
            {
                printf("%.2f ",m[i]);
            }
            printf("\n");
            
            Vec2 res = getEntity()->getAncestorVar<Vec2>("res",Vec2(800,600));
            
            log("res %f %f translate %f %f",res.x,res.y,m[3],m[7]);
            

            glPushMatrix();
            glLoadIdentity();
            glRasterPos2f(m[3],res.y-m[7]-size);
            
            font->Render(getEntity()->getVar<std::string>("text","").c_str());
            
            glPopMatrix();
            
            return Var();
        }
        
    }
}
