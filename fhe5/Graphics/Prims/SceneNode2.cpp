#include "SceneNode2.h"
#include <Graphics/MaterialManager.h>

#include <fhe/math/Vec2.h>
#include <fhe/math/Rot.h>

#include <SDL/SDL_opengl.h>

namespace fhe
{
    namespace Graphics
    {
        FHE_ASPECT(SceneNode2,Aspect);
        
        FHE_FUNC_IMPL(SceneNode2,msg_render2)
        {
            glPushMatrix();
            
            Vec2 pos = getEntity()->getVar<Vec2>("pos",Vec2(0,0));
            Rot rot = getEntity()->getVar<Rot>("rot",Rot());
            Vec2 scale = getEntity()->getVar<Vec2>("scale",Vec2(1,1));
            
            glTranslatef(pos.x,pos.y,0);
            glRotatef(rot.degrees(),0,0,-1);
            glScalef(scale.x,scale.y,1);
            
            MaterialManager::instance().bind(getEntity()->getVar<VarMap>("material",VarMap()));
            
            return Var();
        }
        
        FHE_FUNC_IMPL(SceneNode2,unmsg_render2)
        {
            glPopMatrix();
            
            MaterialManager::instance().unbind();
            
            return Var();
        }
    }
}
