#include "Renderable.h"

namespace gge
{
    namespace Graphics
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
        
        Var Renderable::on_attach( const Var& arg )
        {
            getEntity()->defaultVar<std::string>("renderableParent",getEntity()->getName());
            return Var();
        }
        
        Var Renderable::on_detach( const Var& arg )
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
            return Var();
        }
        
        Var Renderable::set_renderableParent(  const Var& val  )
        {
            on_detach(Var());
            
            EntityPtr parentEntity = getEntity()->getApp()->getEntity( val.get<std::string>(getEntity()->getName()) );
            assert(parentEntity);
            Ogre::SceneNode* parentSceneNode = parentEntity->getVar<Ogre::SceneNode*>("sceneNode",0);
            assert(parentSceneNode);
            
            Ogre::MovableObject* renderable = getEntity()->getVar<Ogre::MovableObject*>("renderable",0);
            
            if ( renderable )
            {
                parentSceneNode->attachObject(renderable);
            }
            return Var();
        }
        
        Var Renderable::set_renderable(  const Var& arg  )
        {
            on_attach(Var());
            return Var();
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
}
