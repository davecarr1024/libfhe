#include "Spinner.h"
#include <fhe/VarMap.h>
#include <fhe/VarList.h>
#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(Spinner,SceneNode2);
        
        FHE_FUNC_IMPL(Spinner,on_attach)
        {
            VarMap defFill, defStroke;
            
            defFill.setVar<Color>("color",Color(0.5,0.5,0.5,1));
            defStroke.setVar<Color>("color",Color(1,1,1,1));
            
            VarMap fill = getEntity()->getVar<VarMap>("fill",defFill),
                stroke = getEntity()->getVar<VarMap>("stroke",defStroke);
            
            m_left = getEntity()->buildChild("left");
            m_left->setVar<Vec2>("pos",Vec2(0,0));
            m_left->setVar<Vec2>("scale",Vec2(0.25,1));
            m_left->setVar<std::string>("text","<");
            m_left->setVar<VarMap>("fill",fill);
            m_left->setVar<VarMap>("stroke",stroke);
            m_left->buildAspect("Graphics/Gui/Button");
            
            m_right = getEntity()->buildChild("right");
            m_right->setVar<Vec2>("pos",Vec2(0.75,0));
            m_right->setVar<Vec2>("scale",Vec2(0.25,1));
            m_right->setVar<std::string>("text",">");
            m_right->setVar<VarMap>("fill",fill);
            m_right->setVar<VarMap>("stroke",stroke);
            m_right->buildAspect("Graphics/Gui/Button");
            
            EntityPtr center = getEntity()->buildChild("center");
            center->setVar<Vec2>("pos",Vec2(0.25,0));
            center->setVar<Vec2>("scale",Vec2(0.5,1));
            center->setVar<VarMap>("material",fill);
            center->buildAspect("Graphics/Prims/Rect");
            
            EntityPtr centerEdge = center->buildChild("edge");
            centerEdge->setVar<VarMap>("material",stroke);
            centerEdge->setVar<bool>("filled",false);
            centerEdge->buildAspect("Graphics/Prims/Rect");
            
            m_text = center->buildChild("text");
            m_text->setVar<Vec2>("pos",Vec2(0.5,0.5));
            m_text->setVar<VarMap>("material",stroke);
            m_text->setVar<std::string>("align","center");
            m_text->buildAspect("Graphics/Prims/Text");
            
            getEntity()->defaultVar<int>("selection",0);
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Spinner,set_selection)
        {
            VarList values = getEntity()->getVar<VarList>("values",VarList());
            int i = arg.get<int>(0), l = values.length();
            
            if ( !values.empty() )
            {
                while ( i < 0 ) i += l;
                i %= l;
                std::string val = values.getVar<std::string>(i);
                m_text->setVar<std::string>("text",val);
            }
            else
            {
                error("no values");
            }
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Spinner,msg_buttonClicked)
        {
            std::string path = arg.get<std::string>();
            if ( path == m_left->getPath() )
            {
                getEntity()->setVar<int>("selection",getEntity()->getVar<int>("selection",0)-1);
            }
            else if ( path == m_right->getPath() )
            {
                getEntity()->setVar<int>("selection",getEntity()->getVar<int>("selection",0)+1);
            }
            
            return Var();
        }
        
    }
}
