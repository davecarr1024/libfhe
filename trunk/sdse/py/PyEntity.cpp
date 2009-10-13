#include "PyEntity.h"
#include "PyScript.h"
#include "core/App.h"

using namespace sdse;

PyEntity::PyEntity(Entity* _entity) : entity(_entity) {}

boost::python::object PyEntity::getVar(std::string varName) {
    return PyScript::varToPythonObject(entity->getVar(varName));
}

boost::python::object PyEntity::getVarWithDefault(std::string varName, boost::python::object defaultVal) {
    return PyScript::varToPythonObject(entity->getVar(varName,PyScript::pythonObjectToVar(defaultVal)));
}

void PyEntity::setVar(std::string varName, boost::python::object val) {
    entity->setVar(varName,PyScript::pythonObjectToVar(val));
}

bool PyEntity::hasVar(std::string varName) {
    return entity->hasVar(varName);
}

boost::python::object PyEntity::addAspect(std::string aspectType, std::string aspectName) {
    Aspect* aspect = entity->addAspect(aspectType,aspectName);
    if (aspect)
        return boost::python::object(aspect->getName());
    else
        return boost::python::object();
}

bool PyEntity::hasAspect(std::string aspectName) {
    return entity->hasAspect(aspectName);
}

void PyEntity::deleteAspect(std::string aspectName) {
    entity->deleteAspect(aspectName);
}

void PyEntity::publish(std::string cmd, boost::python::object pyArgs) {
    entity->publish(cmd,PyScript::pythonListToVarList(pyArgs));
}

void PyEntity::subscribe(std::string cmd) {
    entity->subscribe(cmd);
}

void PyEntity::unsubscribe(std::string cmd) {
    entity->subscribe(cmd);
}

boost::python::object PyEntity::getEntity(std::string name) {
    Entity* ent = entity->getApp()->getEntity(name);
    if (ent)
        return boost::python::object(boost::python::ptr(new PyEntity(ent))); //I think this is a memory leak?
    else
        return boost::python::object();
}

bool PyEntity::hasEntity(std::string name) {
    return entity->getApp()->hasEntity(name);
}

void PyEntity::deleteEntity(std::string name) {
    entity->getApp()->deleteEntity(name);
}

std::string PyEntity::getName() {
    return entity->getName();
}
