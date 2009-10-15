#include "CubeMesh.h"

namespace fhe
{
    
    FHE_ASPECT(CubeMesh);
    
    CubeMesh::CubeMesh() :
        m_entity(0)
    {
        addFunc("set_material",&CubeMesh::set_material,this);
    }
    
    Ogre::MovableObject* CubeMesh::create( Ogre::SceneManager* sceneManager )
    {
        m_entity = sceneManager->createEntity(getPath(),Ogre::SceneManager::PT_CUBE);
        return m_entity;
    }
    
    void CubeMesh::set_material( Var val )
    {
        if ( m_entity && val.is<std::string>() )
        {
            m_entity->setMaterialName(val.get<std::string>());
        }
    }
    
}
