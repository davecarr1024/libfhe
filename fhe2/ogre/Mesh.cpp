#include "Mesh.h"

namespace fhe
{
    
    FHE_NODE_IMPL(Mesh);
    
    Mesh::Mesh()
    {
        addFunc("set_name",&Mesh::set_name,this);
    }
    
    void Mesh::set_name( Var val )
    {
        Ogre::SceneManager* sceneManager = getSceneManager();
        if ( val.is<std::string>() && sceneManager )
        {
            setContent(0);
            setContent(sceneManager->createEntity(getPath(),val.get<std::string>()));
        }
    }
}
