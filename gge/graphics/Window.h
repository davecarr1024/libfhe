#ifndef WINDOW_H
#define WINDOW_H

#include <gge/Aspect.h>

#include <Ogre.h>
#include <OIS.h>
#include <CEGUI.h>
#include <OgreCEGUIRenderer.h>

namespace gge
{
    class FrameListener;
    
    class Window : public Aspect
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
            
            void on_attach();
            void on_detach();
            
            void msg_update(VarMap args);
            
            Var get_root();
            Var get_camera();
            Var get_sceneManager();
            Var get_renderWindow();
            Var get_sceneNode();
            
            Var get_guiSystem();
            Var get_windowManager();
            Var get_window();
    };
    
}

#endif
