#include "SceneNode3.h"
#include <Graphics/MaterialManager.h>
#include <fhe/math/Vec3.h>
#include <fhe/math/Quat.h>
#include <fhe/math/Mat4.h>
#include <SDL/SDL_opengl.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(SceneNode3,Aspect);
        
        FHE_FUNC_IMPL(SceneNode3,msg_render3)
        {
            MaterialManager::instance().bind(getEntity()->getVar<VarMap>("material",VarMap()));
            
            glPushMatrix();
            
            Vec3 pos = getEntity()->getVar<Vec3>("pos",Vec3(0,0,0)),
                scale = getEntity()->getVar<Vec3>("scale",Vec3(1,1,1));
            Quat rot = getEntity()->getVar<Quat>("rot",Quat());
            
            float angle;
            Vec3 axis;
            rot.toAxisAngle(axis,angle);
            
            glTranslatef(pos.x,pos.y,pos.z);
            glRotatef(Math::degrees(angle),axis.x,axis.y,axis.z);
            glScalef(scale.x,scale.y,scale.z);
            
            return Var();
        }
        
        FHE_FUNC_IMPL(SceneNode3,unmsg_render3)
        {
            glPopMatrix();
            MaterialManager::instance().unbind();
            
            return Var();
        }
        
        FHE_FUNC_IMPL(SceneNode3,get_localTransform)
        {
            return Var::build<Mat4>(Mat4::translation(getEntity()->getVar<Vec3>("pos",Vec3(0,0,0))) *
                Mat4::rotation(getEntity()->getVar<Quat>("rot",Quat())) *
                Mat4::scale(getEntity()->getVar<Vec3>("scale",Vec3(1,1,1))));
        }
        
        FHE_FUNC_IMPL(SceneNode3,get_globalTransform)
        {
            return Var::build<Mat4>(getEntity()->getAncestorVar<Mat4>("globalTransform",Mat4::IDENTITY) *
                getEntity()->getVar<Mat4>("localTransform"));
        }
        
        FHE_FUNC_IMPL(SceneNode3,get_inverseGlobalTransform)
        {
            return Var::build<Mat4>(getEntity()->getVar<Mat4>("globalTransform").inverse());
        }
        
        FHE_FUNC_IMPL(SceneNode3,localToGlobal)
        {
            return Var::build<Vec3>(getEntity()->getVar<Mat4>("globalTransform") * arg.get<Vec3>(Vec3()));
        }
        
        FHE_FUNC_IMPL(SceneNode3,globalToLocal)
        {
            return Var::build<Vec3>(getEntity()->getVar<Mat4>("inverseGlobalTransform") * arg.get<Vec3>(Vec3()));
        }
    }
}
