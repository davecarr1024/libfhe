#ifndef WINDOW_H
#define WINDOW_H

#include <gge/Aspect.h>

#include <Ogre.h>
#include <OIS.h>
#include <CEGUI.h>
#include <OgreCEGUIRenderer.h>

namespace gge
{
    namespace Graphics
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
                
                Var on_attach( const Var& arg );
                Var on_detach( const Var& arg );
                
                Var msg_update( const Var& arg );
                
                Var get_root( const Var& arg );
                Var get_camera( const Var& arg );
                Var get_sceneManager( const Var& arg );
                Var get_renderWindow( const Var& arg );
                Var get_sceneNode( const Var& arg );
                
                Var get_guiSystem( const Var& arg );
                Var get_windowManager( const Var& arg );
                Var get_widget( const Var& arg );
        };

    }
}

#endif
