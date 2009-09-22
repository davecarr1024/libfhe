#ifndef OGRE_WINDOW_H
#define OGRE_WINDOW_H

#include <fhe/Node.h>

#include "Ogre.h"
#include "OgreConfigFile.h"

namespace fhe
{
    
    class OgreWindow : public Node
    {
        private:
            Ogre::Root* m_root;
            Ogre::Camera* m_camera;
            Ogre::SceneManager* m_sceneManager;
            Ogre::RenderWindow* m_renderWindow;
        
            bool setup();
            bool configure();
            void setupResources();
            void chooseSceneManager();
            void createCamera();
            void createViewports();
            void loadResources();
            void createFrameListener();

        public:
            OgreWindow( const std::string& type, const std::string& name  );
            ~OgreWindow();
            
            void msg_update(float time);
    };
    
    NODE_DECL(OgreWindow);
    
}

#endif
