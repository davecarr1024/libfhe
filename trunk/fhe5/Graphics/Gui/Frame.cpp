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
        
        FHE_ASPECT(Frame,Widget);
        
        FHE_FUNC_IMPL(Frame,on_attach)
        {
            m_bg = getEntity()->buildChild("bg");
            m_bg->setVar<VarMap>("material",getEntity()->getVar<VarMap>("fill"));
            m_bg->buildAspect("Graphics/Prims/Rect");
            
            EntityPtr bgEdge = m_bg->buildChild("edge");
            bgEdge->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            bgEdge->setVar<bool>("filled",false);
            bgEdge->buildAspect("Graphics/Prims/Rect");
            
            m_titleBar = m_bg->buildChild("titleBar");
            m_titleBar->setVar<Vec2>("scale",Vec2(1,0.1));
            m_titleBar->setVar<VarMap>("material",getEntity()->getVar<VarMap>("fill"));
            m_titleBar->buildAspect("Graphics/Prims/Rect");
            
            EntityPtr titleBarEdge = m_titleBar->buildChild("edge");
            titleBarEdge->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            titleBarEdge->setVar<bool>("filled",false);
            titleBarEdge->buildAspect("Graphics/Prims/Rect");
            
            m_titleText = m_titleBar->buildChild("text");
            m_titleText->setVar<Vec2>("pos",Vec2(0.5,0.5));
            m_titleText->setVar<std::string>("align","center");
            m_titleText->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            m_titleText->setVar<std::string>("text",getEntity()->getVar<std::string>("title",""));
            m_titleText->buildAspect("Graphics/Prims/Text");
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Frame,msg_mouseButtonDown)
        {
            Vec2 pos = arg.get<VarMap>().getVar<Vec2>("pos");
            if ( m_titleBar && m_titleBar->call<bool,Vec2>("collides",pos) )
            {
                getEntity()->setVar<bool>("dragging",true);
                getEntity()->setVar<Vec2>("dragOffset",getEntity()->getVar<Vec2>("pos") - pos);
                m_titleBar->setVar<VarMap>("material",getEntity()->getVar<VarMap>("focusFill"));
            }
            else
            {
                Vec2 localPos = getEntity()->call<Vec2,Vec2>("globalToLocal",pos);
                if ( localPos.x > 0.95 && localPos.x < 1.05 && localPos.y > 0.95 && localPos.y < 1.05 )
                {
                    getEntity()->setVar<bool>("resizing",true);
                    getEntity()->setVar<Vec2>("resizeStartPos",pos);
                    getEntity()->setVar<Vec2>("resizeStartScale",getEntity()->getVar<Vec2>("scale"));
                    m_titleBar->setVar<VarMap>("material",getEntity()->getVar<VarMap>("focusFill"));
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
