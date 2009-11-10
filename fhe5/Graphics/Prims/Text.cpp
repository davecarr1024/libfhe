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
                
            glRasterPos2f(0,-float(size)/m[5]);
            
            font->Render(getEntity()->getVar<std::string>("text","").c_str());
            
            return Var();
        }
        
    }
}
