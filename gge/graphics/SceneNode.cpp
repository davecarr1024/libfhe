#include "SceneNode.h"
#include "OgreUtil.h"

namespace gge
{
    GGE_ASPECT(SceneNode);
    
    SceneNode::SceneNode()
    {
        addFunc("on_attach",&SceneNode::on_attach,this);
        addFunc("on_detach",&SceneNode::on_detach,this);
        addFunc("set_sceneNodeParent",&SceneNode::set_sceneNodeParent,this);
        addFunc("set_pos",&SceneNode::set_pos,this);
        addFunc("get_pos",&SceneNode::get_pos,this);
        addFunc("set_rot",&SceneNode::set_rot,this);
        addFunc("get_rot",&SceneNode::get_rot,this);
        addFunc("set_scale",&SceneNode::set_scale,this);
        addFunc("get_scale",&SceneNode::get_scale,this);
    }
    
    SceneNode::~SceneNode()
    {
/*        on_detach();
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        if ( sceneNode )
        {
            delete sceneNode;
            getEntity()->setVar<Ogre::SceneNode*>("sceneNode",0);
        }*/
    }
    
    void SceneNode::on_attach()
    {
        getEntity()->defaultVar<std::string>("sceneNodeParent","Window");
        getEntity()->defaultVar<Vec3>("pos",Vec3(0,0,0));
        getEntity()->defaultVar<Quat>("rot",Quat());
        getEntity()->defaultVar<Vec3>("scale",Vec3(1,1,1));
    }
    
    void SceneNode::on_detach()
    {
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        if ( sceneNode )
        {
            Ogre::SceneNode* parent = sceneNode->getParentSceneNode();
            if ( parent )
            {
                parent->removeChild(sceneNode);
            }
        }
    }
    
    void SceneNode::set_sceneNodeParent( Var val )
    {
        on_detach();
        
        EntityPtr parentEntity = getEntity()->getApp()->getEntity( val.get<std::string>("Window") );
        assert(parentEntity);
        Ogre::SceneNode* parentSceneNode = parentEntity->getVar<Ogre::SceneNode*>("sceneNode",0);
        assert(parentSceneNode);
        
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        if ( !sceneNode )
        {
            getEntity()->setVar<Ogre::SceneNode*>("sceneNode",parentSceneNode->createChildSceneNode(getPath()));
        }
        else
        {
            parentSceneNode->addChild(sceneNode);
        }
    }
    
    void SceneNode::set_pos( Var val )
    {
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        if ( sceneNode && val.is<Vec3>() )
        {
            sceneNode->setPosition(OgreUtil::fromVec(val.get<Vec3>()));
        }
    }
    
    Var SceneNode::get_pos()
    {
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        return sceneNode ? Var::build<Vec3>(OgreUtil::toVec(sceneNode->getPosition())) : Var();
    }
    
    void SceneNode::set_rot( Var val )
    {
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        if ( sceneNode && val.is<Quat>() )
        {
            sceneNode->setOrientation(OgreUtil::fromQuat(val.get<Quat>()));
        }
    }
    
    Var SceneNode::get_rot()
    {
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        return sceneNode ? Var::build<Quat>(OgreUtil::toQuat(sceneNode->getOrientation())) : Var();
    }
    
    void SceneNode::set_scale( Var val )
    {
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        if ( sceneNode && val.is<Vec3>() )
        {
            sceneNode->setScale(OgreUtil::fromVec(val.get<Vec3>()));
        }
    }
    
    Var SceneNode::get_scale()
    {
        Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
        return sceneNode ? Var::build<Vec3>(OgreUtil::toVec(sceneNode->getScale())) : Var();
    }
}
