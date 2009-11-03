#include "CubeBody.h"
#include "BulletUtil.h"

namespace gge
{
    GGE_ASPECT(CubeBody);
    
    btCollisionShape* CubeBody::makeShape()
    {
        return new btBoxShape(BulletUtil::fromVec(getEntity()->getVar<Vec3>("cubeSize",Vec3(1,1,1))));
    }
    
}
