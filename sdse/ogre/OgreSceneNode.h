#ifndef SDSE_OGRESCENENODE_H
#define SDSE_OGRESCENENODE_H

///sdse_aspect OgreSceneNode

#include "OgreGraphicsServer.h"
#include "core/Aspect.h"

namespace sdse {
    
    class OgreSceneNode : public Aspect {
        private:
            static OgreGraphicsServer* graphicsServer;
            
        protected:
            Ogre::SceneNode* sceneNode;
            Ogre::SceneNode* parentSceneNode;
            Ogre::SceneManager* sceneManager;
            
        public:
            OgreGraphicsServer* getGraphicsServer();
            
            OgreSceneNode(std::string _type, std::string _name);
            
            virtual void onAttach(Entity* ent);
            virtual void onDetach(Entity* ent);
            virtual void onSetVar(std::string varName, Var val);
            
            void updateParent(std::string parentName);
            void updatePosition(Vector3 position);
            void updateRotation(Quaternion rotation);
            void updateScale(Vector3 scale);
            void updateShown(bool shown);
            void updateShowBoundingBox(bool showBoundingBox);
    };
    
}

#endif
