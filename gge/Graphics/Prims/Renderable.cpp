#include "Renderable.h"

namespace gge
{
    GGE_ASPECT(Renderable);
    
    Renderable::Renderable()
    {
        addFunc("on_attach",&Renderable::on_attach,this);
        addFunc("on_detach",&Renderable::on_detach,this);
        addFunc("set_renderableParent",&Renderable::set_renderableParent,this);
        addFunc("set_renderable",&Renderable::set_renderable,this);
    }
    
    Renderable::~Renderable()
    {
/*        on_detach();
        Ogre::MovableObject* renderable = getEntity()->getVar<Ogre::MovableObject*>("renderable",0);
        if ( renderable )
        {
            delete renderable;
            getEntity()->setVar<Ogre::MovableObject*>("renderable",0);
        }*/
    }
    
    void Renderable::on_attach()
    {
        getEntity()->defaultVar<std::string>("renderableParent",getEntity()->getName());
    }
    
    void Renderable::on_detach()
    {
        Ogre::MovableObject* renderable = getEntity()->getVar<Ogre::MovableObject*>("renderable",0);
        if ( renderable )
        {
            Ogre::SceneNode* parent = renderable->getParentSceneNode();
            if ( parent )
            {
                parent->detachObject(renderable);
            }
        }
    }
    
    void Renderable::set_renderableParent( Var val )
    {
        on_detach();
        
        EntityPtr parentEntity = getEntity()->getApp()->getEntity( val.get<std::string>(getEntity()->getName()) );
        assert(parentEntity);
        Ogre::SceneNode* parentSceneNode = parentEntity->getVar<Ogre::SceneNode*>("sceneNode",0);
        assert(parentSceneNode);
        
        Ogre::MovableObject* renderable = getEntity()->getVar<Ogre::MovableObject*>("renderable",0);
        
        if ( renderable )
        {
            parentSceneNode->attachObject(renderable);
        }
    }
    
    void Renderable::set_renderable( Var val )
    {
        on_attach();
    }
    
    Ogre::SceneManager* Renderable::getSceneManager()
    {
        EntityPtr window = getEntity()->getApp()->getEntity("Window");
        assert(window);
        Ogre::SceneManager* sceneManager = window->getVar<Ogre::SceneManager*>("sceneManager",0);
        assert(sceneManager);
        return sceneManager;
    }
    
}
