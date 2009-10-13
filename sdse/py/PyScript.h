///sdse_aspect PyScript
///sdse_lin_includeDir /usr/include/python2.6
///sdse_lib boost_python
///sdse_lib python2.6

#ifndef PYSCRIPT_H
#define PYSCRIPT_H

#include "core/Aspect.h"
#include "PyEntity.h"

#include "boost/python.hpp"

namespace sdse {

    typedef std::map<std::string, boost::python::object> PythonObjectMap;
    
    class PyScript : public Aspect {
        private:
            static boost::python::object mainNamespace, builtins;
            static bool pythonInitialized;
            
            static void initPython();
            static boost::python::object runString(std::string s);
            static void runFile(std::string filename, boost::python::object locals);
            
            static void hasAttr(boost::python::object obj, std::string name);
            
            PyEntity* pyEnt;
            
            PythonObjectMap msgFuncs, setVarFuncs;
            
        public:
            static boost::python::object varToPythonObject(Var var);
            static Var pythonObjectToVar(boost::python::object obj);
            static boost::python::list varListToPythonList(VarList varList);
            static VarList pythonListToVarList(boost::python::object list);

            PyScript(std::string _type, std::string _name);
            
            void onAttach(Entity* ent);
            void onDetach(Entity* ent);
            
            void onSetVar(std::string varName, Var val);
            void onReceiveMessage(std::string cmd, VarList args);
    };
    
}

#endif
