#ifndef RENDERABLE_H
#define RENDERABLE_H

#include <gge/Aspect.h>
#include <Ogre.h>

namespace gge
{
    
    class Renderable : public Aspect
    {
        protected:
            Ogre::SceneManager* getSceneManager();
        
        public:
            Renderable();
            ~Renderable();
            
            void on_attach();
            void on_detach();
            
            void set_renderable( Var val );
            
            void set_parent( Var val );
    };
    
}

#endif
