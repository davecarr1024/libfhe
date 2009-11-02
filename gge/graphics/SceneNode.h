#ifndef SCENENODE_H
#define SCENENODE_H

#include <gge/Aspect.h>

#include "Ogre.h"

namespace gge
{
    
    class SceneNode : public Aspect
    {
        private:
            Ogre::SceneNode* m_sceneNode;
            
        public:
            SceneNode();
            ~SceneNode();
            
            void on_attach();
            void on_detach();
            
            Var get_sceneNode();
            
            void set_parent( Var val );
    };
    
}

#endif
