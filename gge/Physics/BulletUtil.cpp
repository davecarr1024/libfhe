#include "BulletUtil.h"

namespace gge
{
    
    btVector3 BulletUtil::fromVec( const Vec3& v )
    {
        return btVector3(v.x,v.y,v.z);
    }
    
    Vec3 BulletUtil::toVec( const btVector3& v )
    {
        return Vec3(v.x(),v.y(),v.z());
    }
    
    btQuaternion BulletUtil::fromQuat( const Quat& q )
    {
        return btQuaternion(q.x,q.y,q.z,q.w);
    }
    
    Quat BulletUtil::toQuat( const btQuaternion& q )
    {
        return Quat(q.w(),q.x(),q.y(),q.z());
    }
    
}
