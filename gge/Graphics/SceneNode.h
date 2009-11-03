#ifndef SCENENODE_H
#define SCENENODE_H

#include <gge/Aspect.h>

#include "Ogre.h"

namespace gge
{
    
    class SceneNode : public Aspect
    {
        public:
            SceneNode();
            ~SceneNode();
            
            void on_attach();
            void on_detach();
            
            void set_sceneNodeParent( Var val );
            
            void set_pos( Var val );
            Var get_pos();
            
            void set_rot( Var val );
            Var get_rot();
            
            void set_scale( Var val );
            Var get_scale();
    };
    
}

#endif
