#include "CubeMesh.h"

namespace fhe
{
    
    NODE_IMPL(CubeMesh);
    
    CubeMesh::CubeMesh( const std::string& name, const std::string& type ) :
        SceneNode( name, type ),
        m_entity(0)
    {
        addFunc("on_set_material",&CubeMesh::on_set_material,this);
    }
    
    Ogre::MovableObject* CubeMesh::create( Ogre::SceneManager* sceneManager )
    {
        m_entity = sceneManager->createEntity(getPath(),Ogre::SceneManager::PT_CUBE);
        return m_entity;
    }
    
    void CubeMesh::on_set_material( std::string material )
    {
        if ( m_entity )
        {
            m_entity->setMaterialName(material);
        }
    }
    
}
