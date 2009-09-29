#ifndef CUBEMESH_H
#define CUBEMESH_H

#include "SceneNode.h"

namespace fhe
{
    
    class CubeMesh : public SceneNode
    {
        private:
            Ogre::Entity* m_entity;
            
        public:
            CubeMesh();
            
            Ogre::MovableObject* create( Ogre::SceneManager* sceneManager );
            
            void set_material( Var val );
    };
    
    FHE_NODE_DECL(CubeMesh);
    
}

#endif
