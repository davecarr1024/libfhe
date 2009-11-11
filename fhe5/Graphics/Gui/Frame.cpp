#ifndef FRAME_H
#define FRAME_H

#include "Frame.h"

#include <fhe/VarMap.h>
#include <fhe/math/Color.h>
#include <fhe/math/Vec2.h>
#include <fhe/math/Mat3.h>

namespace fhe
{
    namespace Graphics
    {
        
        FHE_ASPECT(Frame,SceneNode2);
        
        FHE_FUNC_IMPL(Frame,on_attach)
        {
            VarMap defFill, defStroke, defDragFill, defDragStroke;
            
            defFill.setVar<Color>("color",Color(0.5,0.5,0.5,1));
            defDragFill.setVar<Color>("color",Color(0.25,0.25,0.25,1));
            defStroke.setVar<Color>("color",Color(1,1,1,1));
            defDragStroke.setVar<Color>("color",Color(0.5,0.5,0.5,1));
            
            VarMap fill = getEntity()->defaultVar<VarMap>("fill",defFill),
                stroke = getEntity()->defaultVar<VarMap>("stroke",defStroke),
                dragFill = getEntity()->defaultVar<VarMap>("dragFill",defDragFill),
                dragStroke = getEntity()->defaultVar<VarMap>("dragStroke",defDragStroke);
            
            m_bg = getEntity()->buildChild("bg");
            m_bg->setVar<VarMap>("material",fill);
            m_bg->buildAspect("Graphics/Prims/Rect");
            
            EntityPtr bgEdge = m_bg->buildChild("edge");
            bgEdge->setVar<VarMap>("material",stroke);
            bgEdge->setVar<bool>("filled",false);
            bgEdge->buildAspect("Graphics/Prims/Rect");
            
            m_titleBar = m_bg->buildChild("titleBar");
            m_titleBar->setVar<Vec2>("scale",Vec2(1,0.1));
            m_titleBar->setVar<VarMap>("material",fill);
            m_titleBar->buildAspect("Graphics/Prims/Rect");
            
            EntityPtr titleBarEdge = m_titleBar->buildChild("edge");
            titleBarEdge->setVar<VarMap>("material",stroke);
            titleBarEdge->setVar<bool>("filled",false);
            titleBarEdge->buildAspect("Graphics/Prims/Rect");
            
            m_titleText = m_titleBar->buildChild("text");
            m_titleText->setVar<Vec2>("pos",Vec2(0.5,0.5));
            m_titleText->setVar<std::string>("align","center");
            m_titleText->setVar<VarMap>("material",stroke);
            m_titleText->setVar<std::string>("text",getEntity()->getVar<std::string>("title",""));
            m_titleText->buildAspect("Graphics/Prims/Text");
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Frame,msg_mouseButtonDown)
        {
            Vec2 pos = arg.get<VarMap>().getVar<Vec2>("pos");
            if ( m_titleBar && m_titleBar->call("collides",Var::build<Vec2>(pos)).get<bool>(false) )
            {
                getEntity()->setVar<bool>("dragging",true);
                getEntity()->setVar<Vec2>("dragOffset",getEntity()->getVar<Vec2>("pos") - pos);
                m_titleBar->setVar<VarMap>("material",getEntity()->getVar<VarMap>("dragFill"));
            }
            else
            {
                Vec2 localPos = getEntity()->call("globalToLocal",Var::build<Vec2>(pos)).get<Vec2>();
                if ( localPos.x > 0.95 && localPos.x < 1.05 && localPos.y > 0.95 && localPos.y < 1.05 )
                {
                    getEntity()->setVar<bool>("resizing",true);
                    getEntity()->setVar<Vec2>("resizeStartPos",pos);
                    getEntity()->setVar<Vec2>("resizeStartScale",getEntity()->getVar<Vec2>("scale"));
                    m_titleBar->setVar<VarMap>("material",getEntity()->getVar<VarMap>("dragFill"));
                }
            }
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Frame,msg_mouseButtonUp)
        {
            getEntity()->setVar<bool>("dragging",false);
            getEntity()->setVar<bool>("resizing",false);
            m_titleBar->setVar<VarMap>("material",getEntity()->getVar<VarMap>("fill"));
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Frame,msg_mouseMotion)
        {
            if ( getEntity()->getVar<bool>("dragging",false) )
            {
                Vec2 mouse = arg.get<Vec2>(),
                    offset = getEntity()->getVar<Vec2>("dragOffset"),
                    pos = mouse + offset;
                getEntity()->setVar<Vec2>("pos",pos);
            }
            else if ( getEntity()->getVar<bool>("resizing",false) )
            {
                Vec2 mouse = arg.get<Vec2>(),
                    startPos = getEntity()->getVar<Vec2>("resizeStartPos"),
                    startScale = getEntity()->getVar<Vec2>("resizeStartScale"),
                    pos = getEntity()->getVar<Vec2>("pos");
                getEntity()->setVar<Vec2>("scale",(mouse - pos) / (startPos - pos ) * startScale);
            }
            
            return Var();
        }
        
    }
}

#endif
