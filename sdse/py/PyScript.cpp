#include "PyScript.h"
#include "PyEntity.h"

#include "core/FileServer.h"

#include "core/math/Vector3.h"
#include "core/math/Vector2.h"
#include "core/math/Quaternion.h"

using namespace sdse;

bool PyScript::pythonInitialized = false;
boost::python::object PyScript::mainNamespace;
boost::python::object PyScript::builtins;

PyScript::PyScript(std::string _type, std::string _name) : Aspect("PyScript",_type,_name), pyEnt(0) {}

void PyScript::initPython() {
    if (!pythonInitialized) {
        pythonInitialized = true;
        Py_Initialize();
        mainNamespace = boost::python::import("__main__").attr("__dict__");
        builtins = boost::python::import("__builtin__");

        printf("initPython\n");
        
        boost::python::object pyEntity = boost::python::class_<PyEntity>("Entity",boost::python::no_init)
            .def("setVar",&PyEntity::setVar)
            .def("getVar",&PyEntity::getVar)
            .def("getVar",&PyEntity::getVarWithDefault)
            .def("hasVar",&PyEntity::hasVar)
            
            .def("addAspect",&PyEntity::addAspect)
            .def("deleteAspect",&PyEntity::deleteAspect)
            .def("hasAspect",&PyEntity::hasAspect)
            
            .def("subscribe",&PyEntity::subscribe)
            .def("unsubscribe",&PyEntity::unsubscribe)
            .def("_publish",&PyEntity::publish)
            
            .def("getEntity",&PyEntity::getEntity)
            .def("deleteEntity",&PyEntity::deleteEntity)
            .def("hasEntity",&PyEntity::hasEntity)
            
            .def("getName",&PyEntity::getName)
        ;
        
        boost::python::dict locals;
        runFile("py/PyWrappers.py",locals);
        locals["setupWrappers"](pyEntity);
        
        mainNamespace["Entity"] = pyEntity;
        
        mainNamespace["Vector3"] = boost::python::class_<Vector3>("Vector3",boost::python::init<>())
            .def(boost::python::init<float,float,float>())
            .def_readwrite("x",&Vector3::x)
            .def_readwrite("y",&Vector3::y)
            .def_readwrite("z",&Vector3::z)
            .def("__repr__",&Vector3::toString)
        ;
        
        mainNamespace["Vector2"] = boost::python::class_<Vector2>("Vector2",boost::python::init<>())
            .def(boost::python::init<float,float>())
            .def_readwrite("x",&Vector2::x)
            .def_readwrite("y",&Vector2::y)
            .def("__repr__",&Vector2::toString)
        ;
        
        mainNamespace["Quaternion"] = boost::python::class_<Quaternion>("Quaternion",boost::python::init<>())
            .def(boost::python::init<float,boost::python::object>())
            .def(boost::python::init<float,float,float,float>())
            .def_readwrite("x",&Quaternion::x)
            .def_readwrite("y",&Quaternion::y)
            .def_readwrite("z",&Quaternion::z)
            .def_readwrite("w",&Quaternion::w)
            .def("__repr__",&Quaternion::toString)
        ;
    }
}

boost::python::object PyScript::runString(std::string s) {
    initPython();
    boost::python::object ret;
    try {
        ret = boost::python::exec(s.c_str(),mainNamespace);
    } catch (boost::python::error_already_set const&) {
        printf("error running python string %s:\n",s.c_str());
        PyErr_Print();
    }
    return ret;
}

void PyScript::runFile(std::string filename, boost::python::object locals) {
    initPython();
    try {
        boost::python::exec_file(filename.c_str(),mainNamespace,locals);
    } catch (boost::python::error_already_set const&) {
        printf("error running python script %s:\n",filename.c_str());
        PyErr_Print();
    }
}

void PyScript::onAttach(Entity* ent) {
    initPython();

    std::string scriptName;
    if (entity->queryVar("pyScript",scriptName)) {
        
        boost::python::dict locals;
        pyEnt = new PyEntity(getEntity());
        boost::python::object pySelf = boost::python::object(boost::python::ptr(pyEnt));
        locals["self"] = pySelf;
        
        runFile(FileServer::getFile(scriptName),locals);
        
        boost::python::object items = locals.attr("items")();
        int numItems = boost::python::extract<int>(items.attr("__len__")());
        for (int i = 0; i < numItems; ++i) {
            std::string itemName = boost::python::extract<std::string>(items[i][0]);
            boost::python::object item = items[i][1];
            if (builtins.attr("callable")(item)) {
                int pos = itemName.find("_");
                if (pos != std::string::npos) {
                    std::string prefix = itemName.substr(0,pos);
                    std::string op = itemName.substr(pos+1);
                    if (prefix == "msg") {
                        entity->subscribe(op);
                        msgFuncs[op] = item;
                    } else if (prefix == "setVar")
                        setVarFuncs[op] = item;
                }
            }
        }
    }
}

void PyScript::onDetach(Entity* ent) {
    msgFuncs.clear();
    setVarFuncs.clear();
    
    if (pyEnt) {
        delete pyEnt;
        pyEnt = 0;
    }
}

void PyScript::onSetVar(std::string varName, Var val) {
    if (setVarFuncs.find(varName) != setVarFuncs.end()) {
        try {
            boost::python::object pySelf = boost::python::object(boost::python::ptr(pyEnt));
            setVarFuncs[varName](pySelf,varToPythonObject(val));
        } catch (boost::python::error_already_set const&) {
            printf("error running setVar func %s:\n",varName.c_str());
            PyErr_Print();
        } 
    }
}

void PyScript::onReceiveMessage(std::string cmd, VarList args) {
    if (msgFuncs.find(cmd) != msgFuncs.end()) {
        printf("receive message %s\n",cmd.c_str());
        try {
            boost::python::object pySelf = boost::python::object(boost::python::ptr(pyEnt));
            msgFuncs[cmd](pySelf,varListToPythonList(args));
        } catch (boost::python::error_already_set const&) {
            printf("error running msg func %s:\n",cmd.c_str());
            PyErr_Print();
        }
    }
}

boost::python::object PyScript::varToPythonObject(Var var) {
    if (boost::any_cast<bool>(&var))
        return boost::python::object(boost::any_cast<bool>(var));
    else if (boost::any_cast<int>(&var))
        return boost::python::object(boost::any_cast<int>(var));
    else if (boost::any_cast<float>(&var))
        return boost::python::object(boost::any_cast<float>(var));
    else if (boost::any_cast<std::string>(&var))
        return boost::python::object(boost::any_cast<std::string>(var));
    else if (boost::any_cast<Vector2>(&var))
        return boost::python::object(boost::any_cast<Vector2>(var));
    else if (boost::any_cast<Vector3>(&var))
        return boost::python::object(boost::any_cast<Vector3>(var));
    else if (boost::any_cast<Quaternion>(&var))
        return boost::python::object(boost::any_cast<Quaternion>(var));
    else
        return boost::python::object();
}

Var PyScript::pythonObjectToVar(boost::python::object obj) {
    std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
    if (type == "bool") {
        bool b = boost::python::extract<bool>(obj);
        return Var(b);
    } else if (type == "int") {
        int b = boost::python::extract<int>(obj);
        return Var(b);
    } else if (type == "float") {
        float b = boost::python::extract<float>(obj);
        return Var(b);
    } else if (type == "str") {
        std::string b = boost::python::extract<std::string>(obj);
        return Var(b);
    } else if (type == "Vector2") {
        float x, y;
        x = boost::python::extract<float>(obj.attr("x"));
        y = boost::python::extract<float>(obj.attr("y"));
        return Vector2(x,y);
    } else if (type == "Vector3") {
        float x, y, z;
        x = boost::python::extract<float>(obj.attr("x"));
        y = boost::python::extract<float>(obj.attr("y"));
        z = boost::python::extract<float>(obj.attr("z"));
        return Vector3(x,y,z);
    } else if (type == "Quaternion") {
        float x, y, z, w;
        x = boost::python::extract<float>(obj.attr("x"));
        y = boost::python::extract<float>(obj.attr("y"));
        z = boost::python::extract<float>(obj.attr("z"));
        w = boost::python::extract<float>(obj.attr("w"));
        return Quaternion(w,x,y,z);
    } else
        return Var();
}

boost::python::list PyScript::varListToPythonList(VarList varList) {
    boost::python::list list;
    for (VarList::iterator i = varList.begin(); i != varList.end(); ++i)
        list.append(varToPythonObject(*i));
    return list;
}

VarList PyScript::pythonListToVarList(boost::python::object list) {
    VarList varList;
    int itemCount = boost::python::extract<int>(list.attr("__len__")());
    for (int i = 0; i < itemCount; ++i)
        varList.push_back(pythonObjectToVar(list[i]));
    return varList;
}
