#include "SceneNode.h"

namespace gge
{
    
    SceneNode::SceneNode() :
        m_sceneNode(0)
    {
    }
    
    SceneNode::~SceneNode()
    {
        on_detach();
        if ( m_sceneNode )
        {
            delete m_sceneNode;
            m_sceneNode = 0;
        }
    }
    
    void SceneNode::on_attach()
    {
        getEntity()->defaultVar<std::string>("parent","Window");
    }
    
    void SceneNode::on_detach()
    {
        if ( m_sceneNode )
        {
            Ogre::SceneNode* parent = m_sceneNode->getParentSceneNode();
            if ( parent )
            {
                parent->removeChild(m_sceneNode);
            }
        }
    }
    
    Var SceneNode::get_sceneNode()
    {
        return Var::build<Ogre::SceneNode*>(m_sceneNode);
    }
    
    void SceneNode::set_parent( Var val )
    {
        on_detach();
        
        EntityPtr parent = getEntity()->getApp()->getEntity( val.get<std::string>("Window") );
        assert(parent);
        Ogre::SceneNode* parentSceneNode = parent->getVar<Ogre::SceneNode*>("sceneNode",0);
        assert(parentSceneNode);
        
        if ( !m_sceneNode )
        {
            m_sceneNode = parentSceneNode->createChildSceneNode();
        }
        else
        {
            parentSceneNode->addChild(m_sceneNode);
        }
    }
    
}
