#ifndef SDSE_OGREMESH_H
#define SDSE_OGREMESH_H

///sdse_aspect OgreMesh

#include "OgreSceneNode.h"

namespace sdse {
    
    class OgreMesh : public OgreSceneNode {
        private:
            Ogre::Entity* ogreEntity;
            
        public:
            OgreMesh(std::string _type, std::string _name);
            
            void onSetVar(std::string varName, Var val);
            void onAttach(Entity* ent);
            
            void updateOgreMeshName(std::string meshName);
    };
    
}

#endif
