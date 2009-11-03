#include "PyAspect.h"
#include "PyEnv.h"

namespace gge
{
    GGE_ASPECT(PyAspect);
    
    PyAspect::PyAspect()
    {
        addFunc("load_script",&PyAspect::load_script,this);
    }
    
    void PyAspect::load_script( TiXmlHandle h )
    {
        TiXmlElement* e = h.ToElement();
        if ( e )
        {
            run( e->GetText() );
        }
    }
    
    void PyAspect::run( const std::string& filename )
    {
        boost::python::dict ns = PyEnv::defaultNamespace();
        Entity* ent = getEntity();
        ns["self"] = toPy();
        PyEnv::run(filename,ns);
    }
    
}
