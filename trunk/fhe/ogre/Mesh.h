#ifndef MESH_H
#define MESH_H

#include "SceneNode.h"

namespace fhe
{
    
    class Mesh : public SceneNode
    {
        public:
            Mesh( const std::string& name, const std::string& type );
    
            Ogre::MovableObject* create( Ogre::SceneManager* sceneManager );
    };
    
    NODE_DECL(Mesh);
    
}

#endif
