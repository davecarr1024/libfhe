#include "OgreGraphicsServer.h"
#include "core/App.h"
#include "core/FileServer.h"

using namespace sdse;

OgreGraphicsServer::OgreGraphicsServer(App* _app) :
    AppListener(_app),
    root(0),
    camera(0),
    sceneManager(0),
    window(0),
    guiSystem(0),
    guiRenderer(0),
    guiWindowManager(0),
    guiSheet(0),
    lastFrameTime(0) {
    frameTime = 1.0 / 60.0;
    setupOk = setup();
}

void OgreGraphicsServer::update(float time, float dtime) {
    if (setupOk && time - lastFrameTime > frameTime) {
        lastFrameTime = time;
        root->renderOneFrame();
    }
}

void OgreGraphicsServer::shutdown() {
}

bool OgreGraphicsServer::setup() {
    std::string pluginFilename = FileServer::getFile("plugins.cfg");
    if (pluginFilename == "") {
        printf("ERROR: couldn't find graphics plugin file\n");
        return false;
    }
    
    root = new Ogre::Root(pluginFilename,"graphics/ogre.cfg","graphics/ogre.log");
    
    setupResources();
    
    if (!configure()) {
        printf("ERROR: couldn't configure ogre\n");
        return false;
    }
    
    chooseSceneManager();
    createCamera();
    createViewports();
    
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    
    loadResources();
    
    setupGui();
    
    return true;
}

bool OgreGraphicsServer::configure() {
    if (root->restoreConfig() || root->showConfigDialog()) {
        window = root->initialise(true);
        return true;
    } else
        return false;
}

void OgreGraphicsServer::chooseSceneManager() {
    sceneManager = root->createSceneManager(Ogre::ST_GENERIC, "SceneManager");
}

void OgreGraphicsServer::createCamera() {
    camera = sceneManager->createCamera("Camera");
    camera->setPosition(Ogre::Vector3(0,100,150));
    camera->lookAt(Ogre::Vector3::ZERO);
    camera->setNearClipDistance(5);
}

void OgreGraphicsServer::createViewports() {
    Ogre::Viewport* vp = window->addViewport(camera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
    camera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}

void OgreGraphicsServer::setupResources() {
    StringList filePaths = FileServer::getFilePaths();
    for (StringList::iterator i = filePaths.begin(); i != filePaths.end(); ++i)
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(*i,"FileSystem");
    
    StringList zips = FileServer::getFilesOfType("zip");
    for (StringList::iterator i = zips.begin(); i != zips.end(); ++i)
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(*i,"Zip");
}

void OgreGraphicsServer::loadResources() {
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

Ogre::Vector3 OgreGraphicsServer::vectorToOgreVector(Vector3 v) {
    return Ogre::Vector3(v.x,v.y,v.z);
}

Vector3 OgreGraphicsServer::ogreVectorToVector(Ogre::Vector3 v) {
    return Vector3(v.x,v.y,v.z);
}

Ogre::Quaternion OgreGraphicsServer::quaternionToOgreQuaternion(Quaternion q) {
    return Ogre::Quaternion(q.w,q.x,q.y,q.z);
}

Quaternion OgreGraphicsServer::ogreQuaternionToQuaternion(Ogre::Quaternion q) {
    return Quaternion(q.w,q.x,q.y,q.z);
}

void OgreGraphicsServer::setupGui() {
    guiRenderer = new CEGUI::OgreCEGUIRenderer(window, Ogre::RENDER_QUEUE_OVERLAY, false, 3000, sceneManager);
    guiSystem = new CEGUI::System(guiRenderer);
    CEGUI::SchemeManager::getSingleton().loadScheme("TaharezLookSkin.scheme");
    guiWindowManager = CEGUI::WindowManager::getSingletonPtr();
    guiSheet = guiWindowManager->createWindow("DefaultGUISheet","Root");
    guiSystem->setGUISheet(guiSheet);
    CEGUI::MouseCursor::getSingleton().setImage(CEGUI::System::getSingleton().getDefaultMouseCursor());
}
