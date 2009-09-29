#ifndef WINDOW_H
#define WINDOW_H

#include <fhe/Node.h>

#include <Ogre.h>
#include <OIS.h>
#include <CEGUI.h>
#include <OgreCEGUIRenderer.h>

namespace fhe
{
    class FrameListener;
    
    class Window : public Node
    {
        private:
            FrameListener* m_frameListener;
            
            Ogre::Root* m_root;
            Ogre::Camera* m_camera;
            Ogre::SceneManager* m_sceneManager;
            Ogre::RenderWindow* m_renderWindow;
            
            CEGUI::System* m_guiSystem;
            CEGUI::OgreCEGUIRenderer* m_guiRenderer;
            CEGUI::WindowManager* m_guiWindowManager;
            CEGUI::Window* m_guiSheet;
        
            bool setup();
            bool configure();
            void setupResources();
            void chooseSceneManager();
            void createCamera();
            void createViewports();
            void loadResources();
            void createFrameListener();
            void setupGui();

        public:
            Window();
            ~Window();
            
            void msg_update(VarMap args);
            
            Ogre::Root* getOgreRoot();
            Ogre::Camera* getCamera();
            Ogre::SceneManager* getSceneManager();
            Ogre::RenderWindow* getRenderWindow();
            Ogre::SceneNode* getSceneNode();
            
            CEGUI::System* getGuiSystem();
            CEGUI::WindowManager* getWindowManager();
            CEGUI::Window* getWindow();
    };
    
    FHE_NODE_DECL(Window);
    
}

#endif
