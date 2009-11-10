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
            int size = getEntity()->getVar<int>("fontSize",14);
            font->FaceSize(size);
            
            GLfloat m[16];
            glGetFloatv(GL_MODELVIEW_MATRIX,m);
            
            glPushMatrix();
            
            glScalef(1.0/m[0],1.0/m[5],1);
            
            std::string text = getEntity()->getVar<std::string>("text","");
            
            FTBBox bb = font->BBox(text.c_str());
            float w = bb.Upper().X(), h = bb.Upper().Y();
            
            std::string align = getEntity()->getVar<std::string>("align","left");
            float x, y;
            if ( align == "left" )
            {
                x = 0;
                y = -h;
            }
            else if ( align == "center" )
            {
                x = -w/2;
                y = -h/2;
            }
            else 
            {
                x = -w;
                y = -h;
            }
                
            glRasterPos2f(x,y);
            
            font->Render(text.c_str());
            
            glPopMatrix();
            
            return Var();
        }
        
    }
}
