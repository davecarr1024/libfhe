#include "Script.h"
#include "Env.h"
#include "PyEntity.h"
#include "PyFunc.h"

namespace gge
{
    namespace Python
    {
        
        GGE_ASPECT(Script);
        
        Script::Script()
        {
            addFunc("runScript",&Script::runScript,this);
            addFunc("load_script",&Script::load_script,this);
        }
        
        Var Script::load_script( const Var& arg )
        {
            return runScript(Var::build<std::string>(arg.get<TiXmlHandle>().ToElement()->GetText()));
        }
        
        Var Script::runScript( const Var& arg )
        {
            boost::python::dict ns = Env::instance().defaultNamespace();
            boost::python::object self = boost::python::object(PyEntity(getEntity()));
            ns["self"] = self;
            Env::instance().runFile(arg.get<std::string>(),ns);
            
            boost::python::object classItems = ns.attr("items")();
            for ( int i = 0; i < boost::python::len(classItems); ++i )
            {
                std::string className = boost::python::extract<std::string>(classItems[i][0]),
                    classType = Env::instance().getType(classItems[i][1]);
                if ( classType == "classobj" )
                {
                    boost::python::object funcItems = classItems[i][1].attr("__dict__").attr("items")();
                    for ( int j = 0; j < boost::python::len(funcItems); ++j )
                    {
                        std::string funcName = boost::python::extract<std::string>(funcItems[j][0]),
                            funcType = Env::instance().getType(funcItems[j][1]);
                        if ( funcType == "function" )
                        {
                            addFunc(funcName,new PyFunc(self,funcItems[j][1]));
                        }
                    }
                }
                else if ( classType == "function" )
                {
                    addFunc(className,new PyFunc(self,classItems[i][1]));
                }
            }
            
            return Var();
        }
    }
}
