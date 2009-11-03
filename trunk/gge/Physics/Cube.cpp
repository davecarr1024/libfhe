#include "Cube.h"
#include "BulletUtil.h"

namespace gge
{
    namespace Physics
    {
        
        GGE_ASPECT(Cube);
        
        btCollisionShape* Cube::makeShape()
        {
            return new btBoxShape(BulletUtil::fromVec(getEntity()->getVar<Vec3>("cubeSize",Vec3(1,1,1))));
        }
        
    }
}
