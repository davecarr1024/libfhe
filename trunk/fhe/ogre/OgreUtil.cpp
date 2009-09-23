#include "OgreUtil.h"

namespace fhe
{
    
    Ogre::Vector3 OgreUtil::VecToOgreVec( const Vec3& v )
    {
        return Ogre::Vector3( v.x, v.y, v.z );
    }
    
    Vec3 OgreUtil::OgreVecToVec( const Ogre::Vector3& v )
    {
        return Vec3( v.x, v.y, v.z );
    }
    
    Ogre::Quaternion OgreUtil::QuatToOgreQuat( const Quat& q )
    {
        return Ogre::Quaternion( q.w, q.x, q.y, q.z );
    }
    
    Quat OgreUtil::OgreQuatToQuat( const Ogre::Quaternion& q )
    {
        return Quat( q.w, q.x, q.y, q.z );
    }
    
}
