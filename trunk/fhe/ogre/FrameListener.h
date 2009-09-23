#ifndef FRAMELISTENER_H
#define FRAMELISTENER_H

#include "Window.h"

namespace fhe
{
    
    class FrameListener : public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener, public OIS::JoyStickListener
    {
        private:
            WindowPtr m_window;
            
            CEGUI::System* m_guiSystem;
            
            OIS::InputManager* m_inputManager;
            OIS::Keyboard* m_keyboard;
            OIS::Mouse* m_mouse;
            OIS::JoyStick* m_joystick;
            
            CEGUI::MouseButton convertButton(OIS::MouseButtonID buttonID);
            
        public:
            FrameListener(WindowPtr window);
            ~FrameListener();
            
            void windowResized(Ogre::RenderWindow* rw);
            void windowClosed(Ogre::RenderWindow* rw);
            
            bool keyPressed(const OIS::KeyEvent& evt);
            bool keyReleased(const OIS::KeyEvent& evt);
            
            bool mouseMoved(const OIS::MouseEvent& evt);
            bool mousePressed(const OIS::MouseEvent& evt, OIS::MouseButtonID button);
            bool mouseReleased(const OIS::MouseEvent& evt, OIS::MouseButtonID button);
            
            bool buttonPressed(const OIS::JoyStickEvent& evt, int button);
            bool buttonReleased(const OIS::JoyStickEvent& evt, int button);
            bool axisMoved(const OIS::JoyStickEvent& evt, int axis);
            
            void msg_update( float time );
    };
    
}

#endif
