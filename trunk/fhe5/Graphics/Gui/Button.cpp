#include "Button.h"
#include <fhe/VarMap.h>
#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(Button,SceneNode2);
        
        FHE_FUNC_IMPL(Button,on_attach)
        {
            VarMap defFill, defStroke;
            
            defFill.setVar<Color>("color",Color(0.5,0.5,0.5,1));
            defFill.setVar<std::string>("texture","");
            defStroke.setVar<Color>("color",Color(1,1,1,1));
            defStroke.setVar<std::string>("texture","");
            
            VarMap fill = getEntity()->getVar<VarMap>("fill",defFill),
                stroke = getEntity()->getVar<VarMap>("stroke",defStroke);
            
            m_bg = getEntity()->buildChild("bg");
            m_bg->setVar<VarMap>("material",fill);
            m_bg->buildAspect("Graphics/Prims/Rect");
            
            EntityPtr border = m_bg->buildChild("border");
            border->setVar<VarMap>("material",stroke);
            border->setVar<bool>("filled",false);
            border->buildAspect("Graphics/Prims/Rect");
            
            EntityPtr text = m_bg->buildChild("text");
            text->setVar<VarMap>("material",stroke);
            text->setVar<std::string>("text",getEntity()->getVar<std::string>("text",""));
            text->setVar<Vec2>("pos",Vec2(0.5,0.5));
            text->setVar<std::string>("align","center");
            text->buildAspect("Graphics/Prims/Text");
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Button,msg_clickUp)
        {
            if ( m_bg && m_bg->getPath() == arg.get<std::string>("") )
            {
                log("clicked");
                getEntity()->getRoot()->publish("buttonClicked",Var::build<std::string>(getEntity()->getPath()));
            }
            return Var();
        }
        
    }
}
