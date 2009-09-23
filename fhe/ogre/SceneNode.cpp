#include "SceneNode.h"

namespace fhe
{
    NODE_IMPL(SceneNode);
    
    SceneNode::SceneNode( const std::string& type, const std::string& name ) :
        Node( type, name ),
        m_sceneNode(0)
    {
        addFunc("getSceneManager",&SceneNode::getSceneManager,this);
        addFunc("getSceneNode",&SceneNode::getSceneNode,this);
        addFunc("on_attach",&SceneNode::on_attach,this);
        addFunc("on_detach",&SceneNode::on_detach,this);
    }
    
    void SceneNode::on_attach()
    {
        on_detach();
        
        Ogre::SceneNode* parentSceneNode = getParentSceneNode();
        if ( !parentSceneNode )
        {
            error("couldn't find parent sceneNode");
        }
        
        if ( !m_sceneNode )
        {
            Ogre::SceneManager* sceneManager = getSceneManager();
            if ( !sceneManager )
            {
                error("couldn't find sceneManager");
            }
            else
            {
                m_sceneNode = parentSceneNode->createChildSceneNode();
                geom(sceneManager,m_sceneNode);
            }
        }
        else
        {
            parentSceneNode->addChild( m_sceneNode );
        }
    }
    
    void SceneNode::on_detach()
    {
        if ( m_sceneNode )
        {
            Ogre::SceneNode* parent = m_sceneNode->getParentSceneNode();
            if ( parent )
            {
                parent->removeChild( m_sceneNode );
            }
        }
    }
    
    Ogre::SceneManager* SceneNode::getSceneManager()
    {
        for ( NodePtr node = getParent(); node; node = node->getParent() )
        {
            if ( node->hasFunc<Ogre::SceneManager*,void>("getSceneManager") )
            {
                return node->callFunc<Ogre::SceneManager*>("getSceneManager");
            }
        }
        return 0;
    }
    
    Ogre::SceneNode* SceneNode::getSceneNode()
    {
        return m_sceneNode;
    }
    
    Ogre::SceneNode* SceneNode::getParentSceneNode()
    {
        for ( NodePtr node = getParent(); node; node = node->getParent() )
        {
            if ( node->hasFunc<Ogre::SceneNode*,void>("getSceneNode") )
            {
                return node->callFunc<Ogre::SceneNode*>("getSceneNode");
            }
        }
        return 0;
    }
}
