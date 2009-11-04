#include "Mesh.h"

namespace gge
{
    namespace Graphics
    {
        GGE_ASPECT(Mesh);
        
        Mesh::Mesh() :
            Renderable()
        {
            addFunc("on_attach",&Mesh::on_attach,this);
            addFunc("set_meshName",&Mesh::set_meshName,this);
        }
        
        Var Mesh::on_attach( const Var& arg )
        {
            getEntity()->defaultVar<std::string>("meshName","");
            return Var();
        }
        
        Var Mesh::set_meshName( const Var& val )
        {
            std::string name = val.get<std::string>("");
            if ( !name.empty() )
            {
                try
                {
                    getEntity()->setVar<Ogre::MovableObject*>("renderable",getSceneManager()->createEntity(getPath(),name));
                }
                catch ( const Ogre::Exception& e )
                {
                    error("couldn't load mesh file %s: %s",name.c_str(),e.getDescription().c_str());
                }
            }
            return Var();
        }

    }
}
