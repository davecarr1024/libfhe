#include "SceneNode2.h"
#include <Graphics/MaterialManager.h>

#include <fhe/math/Vec2.h>
#include <fhe/math/Rot.h>
#include <fhe/math/Mat3.h>

#include <SDL/SDL_opengl.h>

namespace fhe
{
    namespace Graphics
    {
        FHE_ASPECT(SceneNode2,Aspect);
        
        FHE_FUNC_IMPL(SceneNode2,msg_render2)
        {
            glPushMatrix();
            
            Vec2 pos = getEntity()->getVar<Vec2>("pos",Vec2(0,0));
            Rot rot = getEntity()->getVar<Rot>("rot",Rot());
            Vec2 scale = getEntity()->getVar<Vec2>("scale",Vec2(1,1));
            
            glTranslatef(pos.x,pos.y,0);
            glRotatef(rot.degrees(),0,0,-1);
            glScalef(scale.x,scale.y,1);
            
            MaterialManager::instance().bind(getEntity()->getVar<VarMap>("material",VarMap()));
            
            return Var();
        }
        
        FHE_FUNC_IMPL(SceneNode2,unmsg_render2)
        {
            glPopMatrix();
            
            MaterialManager::instance().unbind();
            
            return Var();
        }
        
        FHE_FUNC_IMPL(SceneNode2,get_localTransform)
        {
            Vec2 t = getEntity()->getVar<Vec2>("pos",Vec2(0,0)),
                 s = getEntity()->getVar<Vec2>("scale",Vec2(1,1));
            Rot r = getEntity()->getVar<Rot>("rot",Rot());
            
            Mat3 mt = Mat3::translation(t),
                 ms = Mat3::scale(s),
                 mr = Mat3::rotation(r);
                 
            Mat3 lt = mt * mr * ms;

            return Var::build<Mat3>( lt );
        }
        
        FHE_FUNC_IMPL(SceneNode2,get_globalTransform)
        {
            Mat3 gt = getEntity()->getAncestorVar<Mat3>("globalTransform",Mat3::IDENTITY) *
                getEntity()->getVar<Mat3>("localTransform");
            return Var::build<Mat3>(gt);
        }
        
        FHE_FUNC_IMPL(SceneNode2,get_inverseGlobalTransform)
        {
            Mat3 igt = getEntity()->getVar<Mat3>("globalTransform").inverse();
            return Var::build<Mat3>(igt);
        }
        
        FHE_FUNC_IMPL(SceneNode2,collides)
        {
            Vec2 pos = getEntity()->call<Vec2,Vec2>("globalToLocal",arg.get<Vec2>(Vec2()));
            return Var::build<bool>(getEntity()->call<bool,Vec2>("collTest",pos,false));
        }
        
        FHE_FUNC_IMPL(SceneNode2,globalToLocal)
        {
            return Var::build<Vec2>(getEntity()->getVar<Mat3>("inverseGlobalTransform") * arg.get<Vec2>(Vec2()));
        }
        
        FHE_FUNC_IMPL(SceneNode2,localToGlobal)
        {
            return Var::build<Vec2>(getEntity()->getVar<Mat3>("globalTransform") * arg.get<Vec2>(Vec2()));
        }
        
        FHE_FUNC_IMPL(SceneNode2,msg_mouseButtonDown)
        {
            Vec2 pos = arg.get<VarMap>().getVar<Vec2>("pos");
            if ( getEntity()->call<bool,Vec2>("collides",pos) )
            {
                getEntity()->getRoot()->publish<std::string>("clickDown",getEntity()->getPath());
                return Var::build<bool>(true);
            }
            return Var();
        }

        FHE_FUNC_IMPL(SceneNode2,msg_mouseButtonUp)
        {
            Vec2 pos = arg.get<VarMap>().getVar<Vec2>("pos");
            if ( getEntity()->call<bool,Vec2>("collides",pos) )
            {
                getEntity()->getRoot()->publish<std::string>("clickUp",getEntity()->getPath());
                return Var::build<bool>(true);
            }
            return Var();
        }
    }
}
