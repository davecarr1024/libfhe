#include "Text.h"
#include <Graphics/MaterialManager.h>

namespace fhe
{
    namespace Graphics
    {
        FHE_ASPECT(Text,SceneNode2);
        
        FHE_FUNC_IMPL(Text,msg_render2)
        {
            FTFont* font = MaterialManager::instance().loadFont(getEntity()->getVar<std::string>("fontName","test.ttf"));
            int size = getEntity()->getVar<int>("fontSize",100);
            font->FaceSize(size);
            std::string text = getEntity()->getVar<std::string>("text","").c_str();
            FTBBox bb = font->BBox(text.c_str());
            float w = bb.Upper().X(), 
                h = bb.Upper().Y(), 
                invSize = 1.0 / float(size);
            
            glPushMatrix();
            glScalef(invSize,-invSize,1);
            glTranslatef(0,-h,0);
            font->Render(text.c_str());
            glPopMatrix();
            
            return Var();
        }
        
    }
}
