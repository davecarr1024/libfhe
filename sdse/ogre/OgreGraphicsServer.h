#ifndef SDSE_OGREGRAPHICSSERVER_H
#define SDSE_OGREGRAPHICSSERVER_H

///sdse_lib OgreMain
///sdse_lin_includeDir /usr/local/include/OGRE

///sdse_lib CEGUIBase
///sdse_lib CEGUIOgreRenderer
///sdse_lin_includeDir /usr/local/include/CEGUI

///sdse_lib OIS
///sdse_lin_includeDir /usr/local/include/OIS

#define OIS_DYNAMIC_LIB

#include "Ogre.h"
#include "OIS.h"
#include "CEGUI.h"
#include "OgreCEGUIRenderer.h"

#include "core/AppListener.h"

#include "core/math/Vector3.h"
#include "core/math/Quaternion.h"

namespace sdse {
    
    class OgreGraphicsServer : public AppListener {
        private:
            Ogre::Root* root;
            Ogre::Camera* camera;
            Ogre::SceneManager* sceneManager;
            Ogre::RenderWindow* window;
            
            CEGUI::System* guiSystem;
            CEGUI::OgreCEGUIRenderer* guiRenderer;
            CEGUI::WindowManager* guiWindowManager;
            CEGUI::Window* guiSheet;
            
            bool setupOk;
            
            bool setup();
            bool configure();
            void chooseSceneManager();
            void createCamera();
            void createViewports();
            void setupResources();
            void loadResources();
            void setupGui();
            
            float lastFrameTime, frameTime;
            
        public:
            OgreGraphicsServer(App* app);
            
            void update(float time, float dtime);
            void shutdown();
            
            Ogre::RenderWindow* getWindow() {return window;}
            Ogre::SceneManager* getSceneManager() {return sceneManager;}
            
            CEGUI::System* getGuiSystem() {return guiSystem;}
            CEGUI::WindowManager* getGuiWindowManager() {return guiWindowManager;}
            CEGUI::Window* getGuiSheet() {return guiSheet;}
            
            static Ogre::Vector3 vectorToOgreVector(Vector3 v);
            static Vector3 ogreVectorToVector(Ogre::Vector3 v);
            static Ogre::Quaternion quaternionToOgreQuaternion(Quaternion q);
            static Quaternion ogreQuaternionToQuaternion(Ogre::Quaternion q);
    };
    
}

#endif
