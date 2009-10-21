#include "SceneNode.h"
#include "OgreUtil.h"

namespace fhe
{
    FHE_NODE_IMPL(SceneNode);
    
    SceneNode::SceneNode() :
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
        addFunc("get_worldPos",&SceneNode::get_worldPos,this);
        addFunc("get_worldScale",&SceneNode::get_worldScale,this);
        addFunc("get_worldRot",&SceneNode::get_worldRot,this);
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
                return node->call<Ogre::SceneManager*>("getSceneManager");
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
                return node->call<Ogre::SceneNode*>("getSceneNode");
            }
        }
        return 0;
    }
    
    Ogre::MovableObject* SceneNode::create( Ogre::SceneManager* sceneManager )
    {
        return 0;
    }
    
    void SceneNode::set_pos( Var val )
    {
        if ( m_sceneNode && val.is<Vec3>() )
        {
            m_sceneNode->setPosition( OgreUtil::VecToOgreVec( val.get<Vec3>() ) );
        }
    }
    
    Var SceneNode::get_pos()
    {
        Var val;
        if ( m_sceneNode )
        {
            val.set<Vec3>(OgreUtil::OgreVecToVec( m_sceneNode->getPosition() ));
        }
        return val;
    }
    
    void SceneNode::set_rot( Var val )
    {
        if ( m_sceneNode && val.is<Quat>() )
        {
            m_sceneNode->setOrientation( OgreUtil::QuatToOgreQuat( val.get<Quat>() ) );
        }
    }
    
    Var SceneNode::get_rot()
    {
        Var val;
        if ( m_sceneNode )
        {
            val.set<Quat>(OgreUtil::OgreQuatToQuat( m_sceneNode->getOrientation() ));
        }
        return val;
    }
    
    void SceneNode::set_scale( Var val )
    {
        if ( m_sceneNode && val.is<Vec3>() )
        {
            m_sceneNode->setScale( OgreUtil::VecToOgreVec( val.get<Vec3>() ) );
        }
    }
    
    Var SceneNode::get_scale()
    {
        Var val;
        if ( m_sceneNode )
        {
            val.set<Vec3>(OgreUtil::OgreVecToVec( m_sceneNode->getScale() ));
        }
        return val;
    }
    
    Var SceneNode::get_worldPos()
    {
        Var val;
        if ( m_sceneNode )
        {
            val.set<Vec3>(OgreUtil::OgreVecToVec( m_sceneNode->_getDerivedPosition() ) );
        }
        return val;
    }

    Var SceneNode::get_worldScale()
    {
        Var val;
        if ( m_sceneNode )
        {
            val.set<Vec3>(OgreUtil::OgreVecToVec( m_sceneNode->_getDerivedScale() ) );
        }
        return val;
    }

    Var SceneNode::get_worldRot()
    {
        Var val;
        if ( m_sceneNode )
        {
            val.set<Vec3>(OgreUtil::OgreQuatToQuat( m_sceneNode->_getDerivedOrientation() ) );
        }
        return val;
    }
}
