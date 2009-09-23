#include "Mesh.h"

namespace fhe
{
    
    NODE_IMPL(Mesh);
    
    Mesh::Mesh( const std::string& name, const std::string& type ) : 
        SceneNode(name,type)
    {
    }
    
    Ogre::MovableObject* Mesh::create( Ogre::SceneManager* sceneManager )
    {
        return sceneManager->createEntity(getPath(),getVar<std::string>("name"));
    }
    
}
