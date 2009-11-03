#include "OgreUtil.h"

namespace gge
{
    namespace Graphics
    {
        
        Ogre::Vector3 OgreUtil::fromVec( const Vec3& v )
        {
            return Ogre::Vector3( v.x, v.y, v.z );
        }
        
        Vec3 OgreUtil::toVec( const Ogre::Vector3& v )
        {
            return Vec3( v.x, v.y, v.z );
        }
        
        Ogre::Quaternion OgreUtil::fromQuat( const Quat& q )
        {
            return Ogre::Quaternion( q.w, q.x, q.y, q.z );
        }
        
        Quat OgreUtil::toQuat( const Ogre::Quaternion& q )
        {
            return Quat( q.w, q.x, q.y, q.z );
        }
    }
}
