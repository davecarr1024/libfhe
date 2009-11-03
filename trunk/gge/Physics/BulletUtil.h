#ifndef BULLET_UTIL_H
#define BULLET_UTIL_H

#include "btBulletDynamicsCommon.h"

#include <gge/math/Vec3.h>
#include <gge/math/Quat.h>

namespace gge
{
    
    class BulletUtil
    {
        public:
            static btVector3 fromVec( const Vec3& v );
            static Vec3 toVec( const btVector3& v );
            
            static btQuaternion fromQuat( const Quat& q );
            static Quat toQuat( const btQuaternion& q );
    };
    
}

#endif
