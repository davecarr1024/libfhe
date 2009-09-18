#include "OgreWindow.h"
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
    }
    
    OgreWindow::~OgreWindow()
    {
    }
    
    bool OgreWindow::setup()
    {
        m_root = new Ogre::Root("plugins.cfg","ogre.cfg","ogre.log");
        
        setupResources();
        
        if ( !configure() )
        {
            return false;
        }
        
        chooseSceneManager();
        createCamera();
        createViewports();
        
        loadResources();
    }
    
    bool OgreWindow::configure()
    {
        if ( m_root->showConfigDialog() )
        {
            m_window = m_root->initialise( true );
            return true;
        }
        return false;
    }
    
    
}
