#include "Window.h"
#include "FrameListener.h"
#include "gge/FileSystem.h"
#include <OgreConfigFile.h>

namespace gge
{
    namespace Graphics
    {
        
        GGE_ASPECT(Window);
        
        Window::Window() :
            m_frameListener(0),
            m_root(0),
            m_camera(0),
            m_sceneManager(0),
            m_renderWindow(0)
        {
            addFunc("on_detach",&Window::on_detach,this);
            addFunc("on_attach",&Window::on_attach,this);
            addFunc("msg_update",&Window::msg_update,this);
            addFunc("get_root",&Window::get_root,this);
            addFunc("get_camera",&Window::get_camera,this);
            addFunc("get_sceneManager",&Window::get_sceneManager,this);
            addFunc("get_renderWindow",&Window::get_renderWindow,this);
            addFunc("get_sceneNode",&Window::get_sceneNode,this);
            addFunc("get_guiSystem",&Window::get_guiSystem,this);
            addFunc("get_widget",&Window::get_widget,this);
            addFunc("get_windowManager",&Window::get_windowManager,this);
        }
        
        Window::~Window()
        {
            on_detach(Var());
        }
        
        Var Window::on_detach( const Var& arg )
        {
            if ( m_frameListener )
            {
                delete m_frameListener;
            }
            if ( m_root )
            {
                delete m_root;
            }
            return Var();
        }
        
        Var Window::on_attach( const Var& arg )
        {
            on_detach(Var());
            if (!setup())
            {
                error("Failed to initialize ogre");
            }
            return Var();
        }
        
        Var Window::msg_update( const Var& arg )
        {
            float time = arg.get<VarMap>().getVar<float>("time");
            static float lastTime = time;
            if ( time - lastTime > 1.0 / getEntity()->getVar<float>("fps",60) )
            {
                lastTime = time;
                m_root->renderOneFrame();
            }
            
            if ( m_frameListener )
            {
                m_frameListener->update();
            }
            return Var();
        }
        
        bool Window::setup()
        {
            m_root = new Ogre::Root("Graphics/plugins.cfg","Graphics/ogre.cfg","Graphics/ogre.log");
            
            setupResources();
            
            if ( !configure() )
            {
                return false;
            }
            
            chooseSceneManager();
            createCamera();
            createViewports();
            loadResources();
            setupGui();
            createFrameListener();
            
            return true;
        }
        
        bool Window::configure()
        {
            if ( m_root->restoreConfig() || m_root->showConfigDialog() )
            {
                m_renderWindow = m_root->initialise( true );
                return true;
            }
            return false;
        }
        
        void Window::chooseSceneManager()
        {
            m_sceneManager = m_root->createSceneManager(Ogre::ST_GENERIC, "gge");
        }
        
        void Window::createCamera()
        {
            m_camera = m_sceneManager->createCamera("gge_cam");
            m_camera->setPosition(Ogre::Vector3(0,0,500));
            m_camera->lookAt(Ogre::Vector3(0,0,-300));
        }
        
        void Window::createFrameListener()
        {
            m_frameListener = new FrameListener(this);
        }
        
        void Window::createViewports()
        {
            Ogre::Viewport* vp = m_renderWindow->addViewport(m_camera);
            vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
            m_camera->setAspectRatio(Ogre::Real(vp->getActualWidth())/Ogre::Real(vp->getActualHeight()));
        }
        
        void Window::setupResources()
        {
            std::vector<std::string> dirs = FileSystem::instance().getAllDirs();
            for ( std::vector<std::string>::iterator i = dirs.begin(); i != dirs.end(); ++i )
            {
                Ogre::ResourceGroupManager::getSingleton().addResourceLocation( *i, "FileSystem");
            }
        }
        
        void Window::loadResources()
        {
            Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
        }
        
        void Window::setupGui()
        {
            m_guiRenderer = new CEGUI::OgreCEGUIRenderer(
                m_renderWindow,
                Ogre::RENDER_QUEUE_OVERLAY,
                false,
                3000,
                m_sceneManager);
            m_guiSystem = new CEGUI::System(m_guiRenderer);
            CEGUI::SchemeManager::getSingleton().loadScheme("TaharezLookSkin.scheme");
            m_guiWindowManager = CEGUI::WindowManager::getSingletonPtr();
            m_guiSheet = m_guiWindowManager->createWindow("DefaultGUISheet","root");
            m_guiSystem->setGUISheet(m_guiSheet);
            CEGUI::MouseCursor::getSingleton().setImage(CEGUI::System::getSingleton().getDefaultMouseCursor());
        }
        
        Var Window::get_root( const Var& arg )
        {
            return Var::build<Ogre::Root*>(m_root);
        }
        
        Var Window::get_camera( const Var& arg )
        {
            return Var::build<Ogre::Camera*>(m_camera);
        }
        
        Var Window::get_sceneManager( const Var& arg )
        {
            return Var::build<Ogre::SceneManager*>(m_sceneManager);
        }
        
        Var Window::get_renderWindow( const Var& arg )
        {
            return Var::build<Ogre::RenderWindow*>(m_renderWindow);
        }
        
        Var Window::get_sceneNode( const Var& arg )
        {
            return m_sceneManager ? Var::build<Ogre::SceneNode*>(m_sceneManager->getRootSceneNode()) : Var();
        }
        
        Var Window::get_guiSystem( const Var& arg )
        {
            return Var::build<CEGUI::System*>(m_guiSystem);
        }
        
        Var Window::get_windowManager( const Var& arg )
        {
            return Var::build<CEGUI::WindowManager*>(m_guiWindowManager);
        }
        
        Var Window::get_widget( const Var& arg )
        {
            return Var::build<CEGUI::Window*>(m_guiSheet);
        }
    }
}
