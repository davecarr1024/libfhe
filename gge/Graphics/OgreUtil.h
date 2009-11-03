#ifndef OGRE_UTIL_H
#define OGRE_UTIL_H

#include <Ogre.h>
#include <gge/math/Vec3.h>
#include <gge/math/Quat.h>

namespace gge
{
    namespace Graphics
    {
        
        class OgreUtil
        {
            public:
                static Ogre::Vector3 fromVec( const Vec3& v );
                static Vec3 toVec( const Ogre::Vector3& v );
                static Ogre::Quaternion fromQuat( const Quat& q );
                static Quat toQuat( const Ogre::Quaternion& q );
        };
    }
}

#endif
