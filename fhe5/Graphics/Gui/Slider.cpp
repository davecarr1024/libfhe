#include "Slider.h"

#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>
#include <sstream>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(Slider,SceneNode2);
        
        FHE_FUNC_IMPL(Slider,on_attach)
        {
            VarMap defFill, defStroke, defClickFill, defClickStroke;
            
            defFill.setVar<Color>("color",Color(0.5,0.5,0.5,1));
            defStroke.setVar<Color>("color",Color(1,1,1,1));
            defClickFill.setVar<Color>("color",Color(0.25,0.25,0.25,1));
            defClickStroke.setVar<Color>("color",Color(0.5,0.5,0.5,1));
            
            VarMap fill = getEntity()->defaultVar<VarMap>("fill",defFill),
                stroke = getEntity()->defaultVar<VarMap>("stroke",defStroke),
                clickFill = getEntity()->defaultVar<VarMap>("clickFill",defClickFill),
                clickStroke = getEntity()->defaultVar<VarMap>("clickStroke",defClickStroke);
                
            m_bg = getEntity()->buildChild("bg");
            m_bg->setVar<VarMap>("material",fill);
            m_bg->setVar<Vec2>("pos",Vec2(0,0.5));
            m_bg->setVar<Vec2>("scale",Vec2(1,0.5));
            m_bg->buildAspect("Graphics/Prims/Rect");
            
            EntityPtr bgEdge = m_bg->buildChild("edge");
            bgEdge->setVar<VarMap>("material",stroke);
            bgEdge->setVar<bool>("filled",false);
            bgEdge->buildAspect("Graphics/Prims/Rect");
            
            m_slider = m_bg->buildChild("slider");
            m_slider->setVar<Vec2>("pos",Vec2());
            m_slider->setVar<Vec2>("scale",Vec2(0.1,1));
            m_slider->setVar<VarMap>("material",fill);
            m_slider->buildAspect("Graphics/Prims/Rect");
            
            m_sliderEdge = m_slider->buildChild("edge");
            m_sliderEdge->setVar<VarMap>("material",stroke);
            m_sliderEdge->setVar<bool>("filled",false);
            m_sliderEdge->buildAspect("Graphics/Prims/Rect");
            
            m_text = getEntity()->buildChild("text");
            m_text->setVar<Vec2>("pos",Vec2(0.5,0.25));
            m_text->setVar<std::string>("align","center");
            m_text->setVar<VarMap>("material",stroke);
            m_text->buildAspect("Graphics/Prims/Text");
            
            getEntity()->defaultVar<float>("value",0.5);
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Slider,msg_mouseButtonDown)
        {
            Var pos = arg.get<VarMap>().getRawVar("pos");
            if ( m_slider->call("collides",pos).get<bool>() )
            {
                getEntity()->setVar<bool>("dragging",true);
                getEntity()->setVar<Vec2>("dragOffset",m_bg->call("globalToLocal",pos).get<Vec2>() - m_slider->getVar<Vec2>("pos"));
                m_slider->setVar<VarMap>("material",getEntity()->getVar<VarMap>("clickFill"));
                m_sliderEdge->setVar<VarMap>("material",getEntity()->getVar<VarMap>("clickStroke"));
            }
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Slider,msg_mouseButtonUp)
        {
            getEntity()->setVar("dragging",false);
            m_slider->setVar<VarMap>("material",getEntity()->getVar<VarMap>("fill"));
            m_sliderEdge->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Slider,msg_mouseMotion)
        {
            if ( getEntity()->getVar<bool>("dragging",false) )
            {
                Vec2 mouse = m_bg->call("globalToLocal",arg).get<Vec2>(),
                    offset = getEntity()->getVar<Vec2>("dragOffset"),
                    pos = mouse - offset;
                getEntity()->setVar<float>("value",Math::clamp(pos.x,0,0.9)/0.9);
            }
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Slider,set_value)
        {
            float val = arg.get<float>(0);
            m_slider->setVar<Vec2>("pos",Vec2(val*0.9,0));
            
            float min = getEntity()->getVar<float>("min",0),
                max = getEntity()->getVar<float>("max",10),
                precision = getEntity()->getVar<float>("precision",0.1),
                newVal = min + val * (max - min);
                
            newVal -= Math::fmod(newVal,precision);
            
            std::ostringstream sout;
            sout << newVal;
            m_text->setVar<std::string>("text",getEntity()->getVar<std::string>("prompt","") + " " + sout.str());
            
            return Var();
        }
        
    }
}
