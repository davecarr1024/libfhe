#include "Mesh.h"

namespace fhe
{
    
    NODE_IMPL(Mesh);
    
    Mesh::Mesh( const std::string& name, const std::string& type ) : 
        SceneNode(name,type)
    {
        addFunc("set_name",&Mesh::set_name,this);
    }
    
    void Mesh::set_name( Var val )
    {
        Ogre::SceneManager* sceneManager = getSceneManager();
        if ( sceneManager )
        {
            setContent(0);
            if ( val.is<std::string>() )
            {
                setContent(sceneManager->createEntity(getPath(),val.get<std::string>()));
            }
        }
        else
        {
            log("warning: discarding name");
        }
    }
}
