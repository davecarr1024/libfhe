#include "Window.h"
#include "FrameListener.h"
#include "fhe/FileSystem.h"
#include <OgreConfigFile.h>

namespace fhe
{
    FHE_ASPECT(Window);
    
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
        addFunc("getOgreRoot",&Window::getOgreRoot,this);
        addFunc("getCamera",&Window::getCamera,this);
        addFunc("getSceneManager",&Window::getSceneManager,this);
        addFunc("getRenderWindow",&Window::getRenderWindow,this);
        addFunc("getSceneNode",&Window::getSceneNode,this);
        addFunc("getGuiSystem",&Window::getGuiSystem,this);
        addFunc("getWindow",&Window::getWindow,this);
        addFunc("getWindowManager",&Window::getWindowManager,this);
    }
    
    void Window::on_detach()
    {
        if ( m_frameListener )
        {
            delete m_frameListener;
        }
        if ( m_root )
        {
            delete m_root;
        }
    }
    
    void Window::on_attach()
    {
        log("setup");
        on_detach();
        if (!setup())
        {
            error("Failed to initialize ogre");
        }
    }
    
    void Window::msg_update( VarMap args )
    {
        float time = args.getVar<float>("time",0);
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
    }
    
    bool Window::setup()
    {
        m_root = new Ogre::Root("ogre/plugins.cfg","ogre/ogre.cfg","ogre/ogre.log");
        
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
        m_sceneManager = m_root->createSceneManager(Ogre::ST_GENERIC, "fhe");
    }
    
    void Window::createCamera()
    {
        m_camera = m_sceneManager->createCamera("fhe_cam");
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
    
    Ogre::Root* Window::getOgreRoot()
    {
        return m_root;
    }
    
    Ogre::Camera* Window::getCamera()
    {
        return m_camera;
    }
    
    Ogre::SceneManager* Window::getSceneManager()
    {
        return m_sceneManager;
    }
    
    Ogre::RenderWindow* Window::getRenderWindow()
    {
        return m_renderWindow;
    }
    
    Ogre::SceneNode* Window::getSceneNode()
    {
        return m_sceneManager ? m_sceneManager->getRootSceneNode() : 0;
    }
    
    CEGUI::System* Window::getGuiSystem()
    {
        return m_guiSystem;
    }
    
    CEGUI::WindowManager* Window::getWindowManager()
    {
        return m_guiWindowManager;
    }
    
    CEGUI::Window* Window::getWindow()
    {
        return m_guiSheet;
    }
}
