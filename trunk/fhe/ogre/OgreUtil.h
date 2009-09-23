#ifndef OGRE_UTIL_H
#define OGRE_UTIL_H

#include <Ogre.h>
#include <fhe/math/Vec3.h>
#include <fhe/math/Quat.h>

namespace fhe
{
    
    class OgreUtil
    {
        public:
            static Ogre::Vector3 VecToOgreVec( const Vec3& v );
            static Vec3 OgreVecToVec( const Ogre::Vector3& v );
            static Ogre::Quaternion QuatToOgreQuat( const Quat& q );
            static Quat OgreQuatToQuat( const Ogre::Quaternion& q );
    };
    
}

#endif
