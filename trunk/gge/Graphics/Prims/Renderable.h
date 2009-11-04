#ifndef RENDERABLE_H
#define RENDERABLE_H

#include <gge/Aspect.h>
#include <Ogre.h>

namespace gge
{
    namespace Graphics
    {
    
        class Renderable : public Aspect
        {
            protected:
                Ogre::SceneManager* getSceneManager();
            
            public:
                Renderable();
                ~Renderable();
                
                Var on_attach( const Var& arg );
                Var on_detach( const Var& arg );
                
                Var set_renderable(  const Var& arg );
                
                Var set_renderableParent(  const Var& arg );
        };

    }
}

#endif
