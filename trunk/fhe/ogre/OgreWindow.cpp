#include "OgreWindow.h"
#include "fhe/FileSystem.h"
#include <stdexcept>

namespace fhe
{
    NODE_IMPL(OgreWindow);
    
    OgreWindow::OgreWindow( const std::string& type, const std::string& name ) :
        Node( type, name ),
        m_root(0),
        m_camera(0),
        m_sceneManager(0),
        m_renderWindow(0)
    {
        if (!setup())
        {
            throw std::runtime_error("Failed to initialize ogre");
        }
        
        addFunc("msg_update",&OgreWindow::msg_update,this);
    }
    
    OgreWindow::~OgreWindow()
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
    
    void OgreWindow::msg_update( float time )
    {
        static float lastTime = time;
        if ( time - lastTime > 1.0 / getVar<float>("fps",60) )
        {
            lastTime = time;
            m_root->renderOneFrame();
        }
    }
    
    bool OgreWindow::setup()
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
    
    bool OgreWindow::configure()
    {
        if ( m_root->restoreConfig() || m_root->showConfigDialog() )
        {
            m_renderWindow = m_root->initialise( true );
            return true;
        }
        return false;
    }
    
    void OgreWindow::chooseSceneManager()
    {
        m_sceneManager = m_root->createSceneManager(Ogre::ST_GENERIC, "fhe");
    }
    
    void OgreWindow::createCamera()
    {
        m_camera = m_sceneManager->createCamera("fhe_cam");
        m_camera->setPosition(Ogre::Vector3(0,0,500));
        m_camera->lookAt(Ogre::Vector3(0,0,-300));
    }
    
    void OgreWindow::createFrameListener()
    {
    }
    
    void OgreWindow::createViewports()
    {
        Ogre::Viewport* vp = m_renderWindow->addViewport(m_camera);
        vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
        m_camera->setAspectRatio(Ogre::Real(vp->getActualWidth())/Ogre::Real(vp->getActualHeight()));
    }
    
    void OgreWindow::setupResources()
    {
        std::vector<std::string> dirs = FileSystem::instance().getAllDirs();
        for ( std::vector<std::string>::iterator i = dirs.begin(); i != dirs.end(); ++i )
        {
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation( *i, "FileSystem");
        }
    }
    
    void OgreWindow::loadResources()
    {
        Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    }
}
