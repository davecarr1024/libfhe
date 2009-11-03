#include "FrameListener.h"
#include <gge/VarMap.h>

namespace gge
{
    
    FrameListener::FrameListener( Window* window ) :
        m_window(window),
        m_guiSystem(0),
        m_inputManager(0),
        m_keyboard(0),
        m_mouse(0),
        m_joystick(0)
    {
        m_guiSystem = m_window->getEntity()->getVar<CEGUI::System*>("guiSystem",0);
        
        Ogre::RenderWindow* rw = m_window->getEntity()->getVar<Ogre::RenderWindow*>("renderWindow",0);
        
        Ogre::WindowEventUtilities::addWindowEventListener(rw,this);
        
        size_t windowHnd = 0;
        std::ostringstream windowHndStr;
        OIS::ParamList pl;
        
        rw->getCustomAttribute("WINDOW", &windowHnd);
        windowHndStr << windowHnd;
        pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
        
        #if defined OIS_WIN32_PLATFORM
        pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND" )));
        pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")));
        pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_FOREGROUND")));
        pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_NONEXCLUSIVE")));
        #elif defined OIS_LINUX_PLATFORM
        pl.insert(std::make_pair(std::string("x11_mouse_grab"), std::string("false")));
        pl.insert(std::make_pair(std::string("x11_mouse_hide"), std::string("false")));
        pl.insert(std::make_pair(std::string("x11_keyboard_grab"), std::string("false")));
        pl.insert(std::make_pair(std::string("XAutoRepeatOn"), std::string("true")));
        #endif
        
        m_inputManager = OIS::InputManager::createInputSystem(pl);
    
        m_keyboard = static_cast<OIS::Keyboard*>(m_inputManager->createInputObject(OIS::OISKeyboard, true));
        m_mouse = static_cast<OIS::Mouse*>(m_inputManager->createInputObject(OIS::OISMouse, true));
        try {
            m_joystick = static_cast<OIS::JoyStick*>(m_inputManager->createInputObject(OIS::OISJoyStick, true));
        } catch (const OIS::Exception &e) {
        }
        
        if (m_keyboard)
            m_keyboard->setEventCallback(this);

        if (m_mouse)
            m_mouse->setEventCallback(this);

        if (m_joystick)
            m_joystick->setEventCallback(this);
    }
    
    FrameListener::~FrameListener() {
        if (m_inputManager) {
            if (m_keyboard)
                m_inputManager->destroyInputObject(m_keyboard);
            if (m_mouse)
                m_inputManager->destroyInputObject(m_mouse);
            if (m_joystick)
                m_inputManager->destroyInputObject(m_joystick);
            OIS::InputManager::destroyInputSystem(m_inputManager);
        }
    }

    void FrameListener::update() {
        Ogre::WindowEventUtilities::messagePump();
        
        if (m_keyboard) m_keyboard->capture();
        if (m_mouse) m_mouse->capture();
        if (m_joystick) m_joystick->capture();
    }

    void FrameListener::windowClosed(Ogre::RenderWindow* rw) {
        m_window->log("window closed");
        m_window->getEntity()->getApp()->shutdown();
    }

    bool FrameListener::keyPressed(const OIS::KeyEvent& evt) {
        if (evt.key == OIS::KC_ESCAPE) {
            m_window->log("escape");
            m_window->getEntity()->getApp()->shutdown();
        }
        
        if (m_guiSystem) {
            m_guiSystem->injectKeyDown(evt.key);
            m_guiSystem->injectChar(evt.text);
        }
        
        return true;
    }

    bool FrameListener::keyReleased(const OIS::KeyEvent& evt) {
        if (m_guiSystem) {
            m_guiSystem->injectKeyUp(evt.key);
        }

        return true;
    }

    CEGUI::MouseButton FrameListener::convertButton(OIS::MouseButtonID buttonID)
    {
        switch (buttonID)
        {
        case OIS::MB_Left:
            return CEGUI::LeftButton;

        case OIS::MB_Right:
            return CEGUI::RightButton;

        case OIS::MB_Middle:
            return CEGUI::MiddleButton;

        default:
            return CEGUI::LeftButton;
        }
    }

    bool FrameListener::mouseMoved(const OIS::MouseEvent& evt) {
        if (m_guiSystem) {
            m_guiSystem->injectMousePosition(evt.state.X.abs,evt.state.Y.abs);
        }
        return true;
    }

    bool FrameListener::mousePressed(const OIS::MouseEvent& evt, OIS::MouseButtonID button) {
        if (m_guiSystem)
            m_guiSystem->injectMouseButtonDown(convertButton(button));

        return true;
    }

    bool FrameListener::mouseReleased(const OIS::MouseEvent& evt, OIS::MouseButtonID button) {
        if (m_guiSystem)
            m_guiSystem->injectMouseButtonUp(convertButton(button));

        return true;
    }

    bool FrameListener::buttonPressed(const OIS::JoyStickEvent& evt, int button) {
        return true;
    }

    bool FrameListener::buttonReleased(const OIS::JoyStickEvent& evt, int button) {
        return true;
    }

    bool FrameListener::axisMoved(const OIS::JoyStickEvent& evt, int axis) {
        return true;
    }

    void FrameListener::windowResized(Ogre::RenderWindow* rw) {
        unsigned int width, height, depth;
        int left, top;
        rw->getMetrics(width, height, depth, left, top);

        if (m_mouse) {
            const OIS::MouseState &ms = m_mouse->getMouseState();
            ms.width = width;
            ms.height = height;
        }
    }
}
