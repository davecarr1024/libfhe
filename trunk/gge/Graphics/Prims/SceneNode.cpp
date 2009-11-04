#include "SceneNode.h"
#include "Graphics/OgreUtil.h"

namespace gge
{
    namespace Graphics
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
        
        Var SceneNode::on_attach( const Var& arg )
        {
            getEntity()->defaultVar<std::string>("sceneNodeParent","Window");
            getEntity()->defaultVar<Vec3>("pos",Vec3(0,0,0));
            getEntity()->defaultVar<Quat>("rot",Quat());
            getEntity()->defaultVar<Vec3>("scale",Vec3(1,1,1));
            return Var();
        }
        
        Var SceneNode::on_detach( const Var& arg )
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
            return Var();
        }
        
        Var SceneNode::set_sceneNodeParent( const Var& val  )
        {
            on_detach(Var());
            
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
            return Var();
        }
        
        Var SceneNode::set_pos(  const Var& val  )
        {
            Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
            if ( sceneNode && val.is<Vec3>() )
            {
                sceneNode->setPosition(OgreUtil::fromVec(val.get<Vec3>()));
            }
            return Var();
        }
        
        Var SceneNode::get_pos( const Var& arg )
        {
            Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
            return sceneNode ? Var::build<Vec3>(OgreUtil::toVec(sceneNode->getPosition())) : Var();
        }
        
        Var SceneNode::set_rot(  const Var& val  )
        {
            Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
            if ( sceneNode && val.is<Quat>() )
            {
                sceneNode->setOrientation(OgreUtil::fromQuat(val.get<Quat>()));
            }
            return Var();
        }
        
        Var SceneNode::get_rot( const Var& arg )
        {
            Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
            return sceneNode ? Var::build<Quat>(OgreUtil::toQuat(sceneNode->getOrientation())) : Var();
        }
        
        Var SceneNode::set_scale(  const Var& val  )
        {
            Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
            if ( sceneNode && val.is<Vec3>() )
            {
                sceneNode->setScale(OgreUtil::fromVec(val.get<Vec3>()));
            }
            return Var();
        }
        
        Var SceneNode::get_scale( const Var& arg )
        {
            Ogre::SceneNode* sceneNode = getEntity()->getVar<Ogre::SceneNode*>("sceneNode",0);
            return sceneNode ? Var::build<Vec3>(OgreUtil::toVec(sceneNode->getScale())) : Var();
        }
    }
}
