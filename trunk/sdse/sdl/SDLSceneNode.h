#ifndef SDLSCENENODE_H
#define SDLSCENENODE_H

///sdse_aspect SDLSceneNode

#include "core/Aspect.h"

#include <vector>

namespace sdse {
    
    class SDLGraphicsServer;
    class SDLSceneNode;
    
    typedef std::vector<SDLSceneNode*> SDLSceneNodeList;
    
    class SDLSceneNode : public Aspect {
        private:
            static SDLGraphicsServer* _graphicsServer;
        
        protected:
            SDLGraphicsServer* getGraphicsServer();
            
            SDLSceneNode* parent;
            SDLSceneNodeList children;
            
        public:
            SDLSceneNode(std::string _type, std::string _name);
            
            void onAttach(Entity* ent);
            void onDetach(Entity* ent);
            
            void addChild(SDLSceneNode* child);
            void removeChild(SDLSceneNode* child);
            void attachToParent(SDLSceneNode* _parent);
            void detachFromParent();
            bool hasAncestor(SDLSceneNode* node);
            
            void updateParent(std::string parentName);
            
            void render();
            virtual void geom() {}
    };
    
}

#endif
