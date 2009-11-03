#include "PlaneBody.h"
#include "BulletUtil.h"

namespace gge
{
    GGE_ASPECT(PlaneBody);
    
    btCollisionShape* PlaneBody::makeShape()
    {
        return new btStaticPlaneShape(BulletUtil::fromVec(getEntity()->getVar<Vec3>("normal",Vec3(0,1,0))),0);
    }
    
}
