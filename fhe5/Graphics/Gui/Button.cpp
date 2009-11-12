#include "Button.h"
#include <fhe/VarMap.h>
#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(Button,Widget);
        
        FHE_FUNC_IMPL(Button,on_attach)
        {
            m_bg = getEntity()->buildChild("bg");
            m_bg->setVar<VarMap>("material",getEntity()->getVar<VarMap>("fill"));
            m_bg->buildAspect("Graphics/Prims/Rect");
            
            m_border = m_bg->buildChild("border");
            m_border->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            m_border->setVar<bool>("filled",false);
            m_border->buildAspect("Graphics/Prims/Rect");
            
            m_text = m_bg->buildChild("text");
            m_text->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            m_text->setVar<std::string>("text",getEntity()->getVar<std::string>("text",""));
            m_text->setVar<Vec2>("pos",Vec2(0.5,0.5));
            m_text->setVar<std::string>("align","center");
            m_text->buildAspect("Graphics/Prims/Text");
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Button,msg_clickDown)
        {
            if ( m_bg && m_bg->getPath() == arg.get<std::string>("") )
            {
                getEntity()->setVar<bool>("clicking",true);
                m_bg->setVar<VarMap>("material",getEntity()->getVar<VarMap>("focusFill"));
                m_border->setVar<VarMap>("material",getEntity()->getVar<VarMap>("focusStroke"));
                m_text->setVar<VarMap>("material",getEntity()->getVar<VarMap>("focusStroke"));
            }
            return Var();
        }
        
        FHE_FUNC_IMPL(Button,msg_mouseButtonUp)
        {
            Var pos = arg.get<VarMap>().getRawVar("pos");
            if ( m_bg && m_bg->call("collides",pos).get<bool>() && getEntity()->getVar<bool>("clicking",false) )
            {
                getEntity()->getRoot()->publish("buttonClicked",Var::build<std::string>(getEntity()->getPath()));
            }
            
            getEntity()->setVar<bool>("clicking",false);
            m_bg->setVar<VarMap>("material",getEntity()->getVar<VarMap>("fill"));
            m_border->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            m_text->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            
            return Var();
        }
        
    }
}
