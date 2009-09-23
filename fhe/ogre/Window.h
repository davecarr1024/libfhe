#ifndef WINDOW_H
#define WINDOW_H

#include <fhe/Node.h>

#include <Ogre.h>
#include <OgreConfigFile.h>

namespace fhe
{
    
    class Window : public Node
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
            Window( const std::string& type, const std::string& name  );
            ~Window();
            
            void msg_update(float time);
            
            Ogre::Root* getRoot();
            Ogre::Camera* getCamera();
            Ogre::SceneManager* getSceneManager();
            Ogre::RenderWindow* getRenderWindow();
            Ogre::SceneNode* getSceneNode();
    };
    
    NODE_DECL(Window);
    
}

#endif
