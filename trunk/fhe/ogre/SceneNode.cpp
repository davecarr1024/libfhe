#include "SceneNode.h"
#include "OgreUtil.h"

namespace fhe
{
    NODE_IMPL(SceneNode);
    
    SceneNode::SceneNode( const std::string& type, const std::string& name ) :
        Node( type, name ),
        m_sceneNode(0),
        m_content(0)
    {
        addFunc("getSceneNode",&SceneNode::getSceneNode,this);
        addFunc("on_attach",&SceneNode::on_attach,this);
        addFunc("on_detach",&SceneNode::on_detach,this);
        addFunc("set_pos",&SceneNode::set_pos,this);
        addFunc("get_pos",&SceneNode::get_pos,this);
        addFunc("set_rot",&SceneNode::set_rot,this);
        addFunc("get_rot",&SceneNode::get_rot,this);
        addFunc("set_scale",&SceneNode::set_scale,this);
        addFunc("get_scale",&SceneNode::get_scale,this);
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
                setContent(create( sceneManager ));
            }
        }
        else
        {
            parentSceneNode->addChild( m_sceneNode );
        }
    }
    
    void SceneNode::setContent( Ogre::MovableObject* content )
    {
        if ( m_sceneNode )
        {
            if ( m_content )
            {
                m_sceneNode->detachObject( m_content );
            }
            m_content = content;
            if ( m_content )
            {
                m_sceneNode->attachObject( m_content );
            }
        }
        else
        {
            log("warning: discarding content");
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
    
    Ogre::MovableObject* SceneNode::create( Ogre::SceneManager* sceneManager )
    {
        return 0;
    }
    
    void SceneNode::set_pos( Vec3 pos )
    {
        if ( m_sceneNode )
        {
            m_sceneNode->setPosition( OgreUtil::VecToOgreVec( pos ) );
        }
        else
        {
            log("warning: discarding pos");
        }
    }
    
    Vec3 SceneNode::get_pos()
    {
        return m_sceneNode ? OgreUtil::OgreVecToVec( m_sceneNode->getPosition() ) : Vec3();
    }
    
    void SceneNode::set_rot( Quat rot )
    {
        if ( m_sceneNode )
        {
            m_sceneNode->setOrientation( OgreUtil::QuatToOgreQuat( rot ) );
        }
        else
        {
            log("warning: discarding rot");
        }
    }
    
    void SceneNode::set_scale( Vec3 scale )
    {
        if ( m_sceneNode )
        {
            m_sceneNode->setScale( OgreUtil::VecToOgreVec( scale ) );
        }
        else
        {
            log("warning: discarding scale");
        }
    }
    
    Vec3 SceneNode::get_scale()
    {
        return m_sceneNode ? OgreUtil::OgreVecToVec( m_sceneNode->getScale() ) : Vec3(1,1,1);
    }
    
    Quat SceneNode::get_rot()
    {
        return m_sceneNode ? OgreUtil::OgreQuatToQuat( m_sceneNode->getOrientation() ) : Quat();
    }
}
