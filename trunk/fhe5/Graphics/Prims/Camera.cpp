#include "Camera.h"
#include <fhe/math/Vec3.h>
#include <SDL/SDL_opengl.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(Camera,SceneNode3);
        
        FHE_FUNC_IMPL(Camera,msg_render3)
        {
            Vec3 pos = getEntity()->getVar<Vec3>("pos",Vec3(10,10,10)),
                lookAt = getEntity()->getVar<Vec3>("lookAt",Vec3(0,0,0)),
                up = getEntity()->getVar<Vec3>("up",Vec3(0,1,0));
                
            gluLookAt(pos.x,pos.y,pos.z,lookAt.x,lookAt.y,lookAt.z,up.x,up.y,up.z);
            
            return Var();
        }
    }
}
