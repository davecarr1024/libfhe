#include "Radio.h"
#include <fhe/math/Vec2.h>
#include <fhe/math/Color.h>

namespace fhe
{
    namespace Graphics
    {
        FHE_ASPECT(Radio,Widget);
        
        FHE_FUNC_IMPL(Radio,on_attach)
        {
            m_circle = getEntity()->buildChild("circle");
            m_circle->setVar<Vec2>("pos",Vec2(0.1,0.5));
            m_circle->setVar<Vec2>("scale",Vec2(0.2,1));
            m_circle->setVar<VarMap>("material",getEntity()->getVar<VarMap>("fill"));
            m_circle->buildAspect("Graphics/Prims/Circle");
            
            m_circleEdge = m_circle->buildChild("edge");
            m_circleEdge->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            m_circleEdge->setVar<bool>("filled",false);
            m_circleEdge->buildAspect("Graphics/Prims/Circle");
            
            m_text = getEntity()->buildChild("text");
            m_text->setVar<Vec2>("pos",Vec2(0.25,0.5));
            m_text->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            m_text->setVar<std::string>("text",getEntity()->getVar<std::string>("text",""));
            m_text->buildAspect("Graphics/Prims/Text");
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Radio,msg_mouseButtonDown)
        {
            Vec2 pos = arg.get<VarMap>().getVar<Vec2>("pos");
            if ( m_circle->call<bool,Vec2>("collides",pos) )
            {
                getEntity()->setVar<bool>("clicking",true);
                m_circle->setVar<VarMap>("material",getEntity()->getVar<VarMap>("focusFill"));
                m_circleEdge->setVar<VarMap>("material",getEntity()->getVar<VarMap>("focusStroke"));
                m_text->setVar<VarMap>("material",getEntity()->getVar<VarMap>("focusStroke"));
            }
            
            return Var();
        }
        
        FHE_FUNC_IMPL(Radio,msg_mouseButtonUp)
        {
            Vec2 pos = arg.get<VarMap>().getVar<Vec2>("pos");
            if ( getEntity()->getVar<bool>("clicking",false) && m_circle->call<bool,Vec2>("collides",pos) )
            {
                getEntity()->setVar<bool>("selected",true);
            }

            getEntity()->setVar<bool>("clicking",false);
            m_circle->setVar<VarMap>("material",getEntity()->getVar<VarMap>("fill"));
            m_circleEdge->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
            m_text->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));

            return Var();
        }
        
        FHE_FUNC_IMPL(Radio,set_selected)
        {
            if ( arg.get<bool>(false) )
            {
                EntityPtr parent = getEntity()->getParent();
                if ( parent )
                {
                    std::vector<std::string> names = parent->getChildNames();
                    for ( std::vector<std::string>::iterator i = names.begin(); i != names.end(); ++i )
                    {
                        EntityPtr child = parent->getChild(*i);
                        if ( child->getPath() != getPath() )
                        {
                            child->setVar<bool>("selected",false);
                        }
                    }
                }
                
                m_highlight = m_circle->buildChild("highlight");
                m_highlight->setVar<Vec2>("scale",Vec2(0.5,0.5));
                m_highlight->setVar<VarMap>("material",getEntity()->getVar<VarMap>("stroke"));
                m_highlight->buildAspect("Graphics/Prims/Circle");
            }
            else
            {
                if ( m_highlight )
                {
                    m_highlight->detachFromParent();
                    m_highlight = 0;
                }
            }
            
            return Var();
        }
    }
}
