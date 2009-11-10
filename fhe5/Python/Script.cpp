#include "Script.h"
#include "PyEnv.h"
#include "PyEntity.h"
#include "PyFunc.h"

namespace fhe
{
    namespace Python
    {
        FHE_ASPECT(Script,Aspect);
        
        FHE_FUNC_IMPL(Script,load_script)
        {
            getEntity()->call("runScript",Var::build<std::string>(arg.get<TiXmlHandle>().ToElement()->GetText()));
            return Var();
        }
        
        FHE_FUNC_IMPL(Script,runScript)
        {
            boost::python::dict ns = PyEnv::instance().defaultNamespace();
            boost::python::object self = boost::python::object(PyEntity(getEntity().get()));
            ns["self"] = self;
            PyEnv::instance().runFile(arg.get<std::string>(),ns);
            
            boost::python::object items = ns.attr("items")();
            for ( int i = 0; i < boost::python::len(items); ++i )
            {
                std::string name = boost::python::extract<std::string>(items[i][0]),
                    type = PyEnv::instance().getType(items[i][1]);
                if ( type == "function" )
                {
                    addFunc(new PyFunc(name,self,items[i][1]));
                }
            }
            
            return Var();
        }
        
    }
}
