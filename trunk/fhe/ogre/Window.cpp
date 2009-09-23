#include "Window.h"
#include "fhe/FileSystem.h"
#include <stdexcept>

namespace fhe
{
    NODE_IMPL(Window);
    
    Window::Window( const std::string& type, const std::string& name ) :
        Node( type, name ),
        m_root(0),
        m_camera(0),
        m_sceneManager(0),
        m_renderWindow(0)
    {
        if (!setup())
        {
            error("Failed to initialize ogre");
        }
        
        addFunc("msg_update",&Window::msg_update,this);
        addFunc("getRoot",&Window::getRoot,this);
        addFunc("getCamera",&Window::getCamera,this);
        addFunc("getSceneManager",&Window::getSceneManager,this);
        addFunc("getRenderWindow",&Window::getRenderWindow,this);
        addFunc("getSceneNode",&Window::getSceneNode,this);
    }
    
    Window::~Window()
    {
/*        if ( m_frameListener )
        {
            delete m_frameListener;
        }*/
        if ( m_root )
        {
            delete m_root;
        }
    }
    
    void Window::msg_update( float time )
    {
        static float lastTime = time;
        if ( time - lastTime > 1.0 / getVar<float>("fps",60) )
        {
            lastTime = time;
            m_root->renderOneFrame();
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
    
    Ogre::Root* Window::getRoot()
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
}
