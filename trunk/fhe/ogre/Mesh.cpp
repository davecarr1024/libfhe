#include "Mesh.h"

namespace fhe
{
    
    NODE_IMPL(Mesh);
    
    Mesh::Mesh( const std::string& name, const std::string& type ) : 
        SceneNode(name,type)
    {
        addFunc("on_set_name",&Mesh::on_set_name,this);
    }
    
    void Mesh::on_set_name( std::string name )
    {
        Ogre::SceneManager* sceneManager = getSceneManager();
        if ( sceneManager )
        {
            setContent(0);
            setContent(sceneManager->createEntity(getPath(),name));
        }
        else
        {
            log("warning: discarding name");
        }
    }
}
