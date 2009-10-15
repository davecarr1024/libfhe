#include "Mesh.h"

namespace fhe
{
    
    FHE_ASPECT(Mesh);
    
    Mesh::Mesh()
    {
        addFunc("set_name",&Mesh::set_name,this);
    }
    
    void Mesh::set_name( Var val )
    {
        Ogre::SceneManager* sceneManager = getSceneManager();
        if ( val.is<std::string>() && sceneManager )
        {
            log("load model %s",val.get<std::string>().c_str());
            setContent(0);
            setContent(sceneManager->createEntity(getPath(),val.get<std::string>()));
        }
        else
        {
            log("warning: discarding name");
        }
    }
}
