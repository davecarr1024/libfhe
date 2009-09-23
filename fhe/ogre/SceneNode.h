#ifndef SCENENODE_H
#define SCENENODE_H

#include <fhe/Node.h>

#include <Ogre.h>

namespace fhe
{
    class SceneNode : public Node
    {
        private:
            Ogre::SceneNode* m_sceneNode;
            
        public:
            SceneNode( const std::string& type, const std::string& name );
            
            void on_attach();
            void on_detach();
            
            virtual void geom( Ogre::SceneManager* sceneManager, Ogre::SceneNode* sceneNode ) {}
            
            Ogre::SceneManager* getSceneManager();
            Ogre::SceneNode* getSceneNode();
            Ogre::SceneNode* getParentSceneNode();
    };
    
    NODE_DECL(SceneNode);
}

#endif
