#include "Mesh.h"

namespace gge
{
    GGE_ASPECT(Mesh);
    
    Mesh::Mesh() :
        Renderable()
    {
        addFunc("on_attach",&Mesh::on_attach,this);
        addFunc("set_name",&Mesh::set_name,this);
    }
    
    void Mesh::on_attach()
    {
        getEntity()->defaultVar<std::string>("name","robot.mesh");
    }
    
    void Mesh::set_name( Var val )
    {
        std::string name = val.get<std::string>("robot.mesh");
        log("load %s",name.c_str());
        getEntity()->setVar<Ogre::MovableObject*>("renderable",getSceneManager()->createEntity(getPath(),name));
    }
    
}
