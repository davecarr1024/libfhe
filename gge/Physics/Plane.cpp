#include "Plane.h"
#include "BulletUtil.h"

namespace gge
{
    namespace Physics
    {
        GGE_ASPECT(Plane);
        
        btCollisionShape* Plane::makeShape()
        {
            return new btStaticPlaneShape(BulletUtil::fromVec(getEntity()->getVar<Vec3>("normal",Vec3(0,1,0))),0);
        }
        
    }
}
