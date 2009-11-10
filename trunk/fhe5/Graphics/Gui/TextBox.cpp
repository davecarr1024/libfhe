#include "TextBox.h"
#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(TextBox,SceneNode2);
        
        FHE_FUNC_IMPL(TextBox,on_attach)
        {
            VarMap defFocusStroke, defUnfocusStroke, defFocusFill, defUnfocusFill;

            defFocusStroke.setVar<Color>("color",Color(1,1,1,1));
            defUnfocusStroke.setVar<Color>("color",Color(0.5,0.5,0.5,1));
            defFocusFill.setVar<Color>("color",Color(0.5,0.5,0.5,1));
            defUnfocusFill.setVar<Color>("color",Color(0.25,0.25,0.25,1));

            getEntity()->defaultVar<VarMap>("focusFill",defFocusFill);
            getEntity()->defaultVar<VarMap>("unfocusFill",defUnfocusFill);
            getEntity()->defaultVar<VarMap>("focusStroke",defFocusStroke);
            getEntity()->defaultVar<VarMap>("unfocusStroke",defUnfocusStroke);

            m_bg = getEntity()->buildChild("bg");
            m_bg->buildAspect("Graphics/Prims/Rect");
            
            m_edge = m_bg->buildChild("edge");
            m_edge->setVar<bool>("filled",false);
            m_edge->buildAspect("Graphics/Prims/Rect");
            
            m_text = m_bg->buildChild("text");
            m_text->setVar<std::string>("align","center");
            m_text->setVar<Vec2>("pos",Vec2(0.5,0.5));
            m_text->buildAspect("Graphics/Prims/Text");
            
            updateFocus();
                
            return Var();
        }
        
        void TextBox::updateFocus()
        {
            bool focused = getEntity()->getVar<bool>("focused",false);
            if ( m_bg )
            {
                m_bg->setVar<VarMap>("material", focused ?
                    getEntity()->getVar<VarMap>("focusFill") : getEntity()->getVar<VarMap>("unfocusFill"));
            }
            if ( m_edge )
            {
                m_edge->setVar<VarMap>("material", focused ?
                    getEntity()->getVar<VarMap>("focusStroke") : getEntity()->getVar<VarMap>("unfocusStroke"));
            }
            if ( m_text )
            {
                m_text->setVar<VarMap>("material", focused ?
                    getEntity()->getVar<VarMap>("focusStroke") : getEntity()->getVar<VarMap>("unfocusStroke"));
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
                    std::string text = m_text->getVar<std::string>("text","");
                    m_text->setVar<std::string>("text",text.substr(0,text.size()-1));
                }
                else if ( key == 13 )
                {
                    VarMap args;
                    args.setVar<std::string>("path",getEntity()->getPath());
                    args.setVar<std::string>("text",m_text->getVar<std::string>("text",""));
                    getEntity()->getRoot()->publish("textBoxAccepted",Var::build<VarMap>(args));
                }
                else if ( isprint(key) )
                {
                    m_text->setVar<std::string>("text",m_text->getVar<std::string>("text","") + key);
                }
            }
                
            return Var();
        }
        
    }
}
