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
            Mat3 gt = getEntity()->getVar<Mat3>("localTransform") * 
                getEntity()->getAncestorVar<Mat3>("globalTransform",Mat3::IDENTITY);
            return Var::build<Mat3>(gt);
        }
        
        FHE_FUNC_IMPL(SceneNode2,get_inverseGlobalTransform)
        {
            Mat3 igt = getEntity()->getVar<Mat3>("globalTransform").inverse();
            return Var::build<Mat3>(igt);
        }
        
        FHE_FUNC_IMPL(SceneNode2,msg_mouseButtonDown)
        {
            Mat3 igt = getEntity()->getVar<Mat3>("inverseGlobalTransform");
            Vec2 pos = arg.get<VarMap>().getVar<Vec2>("pos"), tpos = igt * pos;
            if ( getEntity()->call("collTest",Var::build<Vec2>(tpos)).get<bool>(false) )
            {
                getEntity()->getRoot()->publish("clickDown",Var::build<std::string>(getEntity()->getPath()));
                return Var::build<bool>(true);
            }
            return Var();
        }

        FHE_FUNC_IMPL(SceneNode2,msg_mouseButtonUp)
        {
            Mat3 igt = getEntity()->getVar<Mat3>("inverseGlobalTransform");
            Vec2 pos = arg.get<VarMap>().getVar<Vec2>("pos"), tpos = igt * pos;
            if ( getEntity()->call("collTest",Var::build<Vec2>(tpos)).get<bool>(false) )
            {
                getEntity()->getRoot()->publish("clickUp",Var::build<std::string>(getEntity()->getPath()));
                return Var::build<bool>(true);
            }
            return Var();
        }
    }
}
