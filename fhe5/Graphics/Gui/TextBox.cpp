#include "TextBox.h"
#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(TextBox,Widget);
        
        FHE_FUNC_IMPL(TextBox,on_attach)
        {
            m_bg = getEntity()->buildChild("bg");
            m_bg->buildAspect("Graphics/Prims/Rect");
            
            m_edge = m_bg->buildChild("edge");
            m_edge->setVar<bool>("filled",false);
            m_edge->buildAspect("Graphics/Prims/Rect");
            
            m_text = m_bg->buildChild("text");
            m_text->setVar<std::string>("align","center");
            m_text->setVar<Vec2>("pos",Vec2(0.5,0.5));
            m_text->buildAspect("Graphics/Prims/Text");
            
            getEntity()->defaultVar<std::string>("text","");
            
            updateFocus();
                
            return Var();
        }
        
        void TextBox::updateFocus()
        {
            bool focused = getEntity()->getVar<bool>("focused",false);
            if ( m_bg )
            {
                m_bg->setVar<VarMap>("material", focused ?
                    getEntity()->getVar<VarMap>("focusFill") : getEntity()->getVar<VarMap>("fill"));
            }
            if ( m_edge )
            {
                m_edge->setVar<VarMap>("material", focused ?
                    getEntity()->getVar<VarMap>("focusStroke") : getEntity()->getVar<VarMap>("stroke"));
            }
            if ( m_text )
            {
                m_text->setVar<VarMap>("material", focused ?
                    getEntity()->getVar<VarMap>("focusStroke") : getEntity()->getVar<VarMap>("stroke"));
            }
        }
        
        FHE_FUNC_IMPL(TextBox,msg_mouseButtonDown)
        {
            getEntity()->setVar<bool>("focused",m_bg->call("msg_mouseButtonDown",arg).get<bool>(false));
            updateFocus();
            return Var();
        }
        
        FHE_FUNC_IMPL(TextBox,msg_keyDown)
        {
            if ( getEntity()->getVar<bool>("focused",false) )
            {
                char key = arg.get<int>();
                
                if ( key == 8 )
                {
                    std::string text = getEntity()->getVar<std::string>("text","");
                    getEntity()->setVar<std::string>("text",text.substr(0,text.size()-1));
                }
                else if ( key == 13 )
                {
                    VarMap args;
                    args.setVar<std::string>("path",getEntity()->getPath());
                    args.setVar<std::string>("text",getEntity()->getVar<std::string>("text",""));
                    getEntity()->getRoot()->publish("textBoxAccepted",Var::build<VarMap>(args));
                }
                else if ( isprint(key) )
                {
                    getEntity()->setVar<std::string>("text",getEntity()->getVar<std::string>("text","") + key);
                }
            }
                
            return Var();
        }
        
        FHE_FUNC_IMPL(TextBox,set_text)
        {
            m_text->setVar<std::string>("text",getEntity()->getVar<std::string>("prompt","") + " " + arg.get<std::string>());
            return Var();
        }
        
    }
}
