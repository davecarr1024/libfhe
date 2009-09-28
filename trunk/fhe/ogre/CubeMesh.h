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
            CubeMesh( const std::string& name, const std::string& type );
            
            Ogre::MovableObject* create( Ogre::SceneManager* sceneManager );
            
            void set_material( Var material );
    };
    
    NODE_DECL(CubeMesh);
    
}

#endif
