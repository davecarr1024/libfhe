#include "Script.h"
#include "PyEnv.h"
#include "PyEntity.h"

namespace fhe
{
    namespace Python
    {
        FHE_ASPECT(Script,Aspect);
        
        FHE_FUNC_IMPL(Script,load_script)
        {
            getEntity()->callNoRet<std::string>("runScript",arg.get<TiXmlHandle>().ToElement()->GetText());
            return Var();
        }
        
        FHE_FUNC_IMPL(Script,runScript)
        {
            boost::python::dict ns = PyEnv::instance().defaultNamespace();
            ns["self"] = PyEntity(getEntity().get(),this);
            PyEnv::instance().runFile(arg.get<std::string>(),ns);
            return Var();
        }
        
    }
}
