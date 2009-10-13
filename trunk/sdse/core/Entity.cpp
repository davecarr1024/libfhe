#include "Entity.h"
#include "App.h"

#include <sstream>
#include <cstring>
#include <cstdio>
#include <cstdarg>

using namespace sdse;

EntityMap Entity::entities;
EntityListMap Entity::subscribers;

Entity::Entity(std::string _name) :
    app(0) {
    if (entities.find(_name) == entities.end()) {
        name = _name;
    } else {
        std::ostringstream sout;
        int i = 2;
        do {
            sout << _name << "_" << i;
            name = sout.str();
            sout.str("");
            i++;
        } while (entities.find(name) != entities.end());
    }
    entities[name] = this;
}

Entity::~Entity() {
    entities.erase(name);
    for (AspectMap::iterator i = aspects.begin(); i != aspects.end(); ++i) {
        i->second->detach();
        delete i->second;
    }
    detach();
}

void Entity::attach(App* _app) {
    if (_app != app) {
        detach();
        app = _app;
        app->addEntity(this);
        log("attach");
    }
}

void Entity::detach() {
    if (app) {
        App* _app = app;
        app = 0;
        _app->removeEntity(this);
        log("detach");
    }
}

App* Entity::getApp() {
    return app;
}

std::string Entity::getName() {
    return name;
}

void Entity::addAspect(Aspect* aspect) {
    if (!hasAspect(aspect->getName())) {
        aspects[aspect->getName()] = aspect;
        aspect->attach(this);
    }
}

Aspect* Entity::addAspect(std::string aspectType, std::string aspectName) {
    Aspect* aspect = aspectFactory(aspectType, aspectName);
    if (aspect)
        addAspect(aspect);
    return aspect;
}

void Entity::removeAspect(Aspect* aspect) {
    if (hasAspect(aspect->getName())) {
        aspects.erase(aspect->getName());
        aspect->detach();
    }
}

void Entity::deleteAspect(std::string name) {
    Aspect* aspect = getAspect(name);
    if (aspect) {
        removeAspect(aspect);
        delete aspect;
    }
}

bool Entity::hasAspect(std::string name) {
    return aspects.find(name) != aspects.end();
}

Aspect* Entity::getAspect(std::string name) {
    AspectMap::iterator i = aspects.find(name);
    if (i != aspects.end())
        return i->second;
    else
        return 0;
}

Aspect* Entity::getFirstAspectOfArchetype(std::string archetype) {
    for (AspectMap::iterator i = aspects.begin(); i != aspects.end(); ++i)
        if (i->second->getArchetype() == archetype)
            return i->second;
    return 0;
}

AspectList Entity::getAllAspectsOfArchetype(std::string archetype) {
    AspectList aspectList;
    for (AspectMap::iterator i = aspects.begin(); i != aspects.end(); ++i)
        if (i->second->getArchetype() == archetype)
            aspectList.push_back(i->second);
    return aspectList;
}

bool Entity::hasVar(std::string varName) {
    return vars.find(varName) != vars.end();
}

Var Entity::getVar(std::string varName) {
    return getVar(varName, Var());
}

Var Entity::getVar(std::string varName, Var defaultVal) {
    VarMap::iterator i = vars.find(varName);
    if (i != vars.end())
        return i->second;
    else
        return defaultVal;
}

void Entity::setVar(std::string varName, Var val) {
    vars[varName] = val;
    
    for (AspectMap::iterator i = aspects.begin(); i != aspects.end(); ++i)
        i->second->onSetVar(varName,val);
}

void Entity::receiveMessage(std::string cmd, VarList args) {
    for (AspectMap::iterator i = aspects.begin(); i != aspects.end(); ++i)
        i->second->onReceiveMessage(cmd,args);
}

void Entity::subscribe(std::string cmd) {
    if (subscribers.find(cmd) == subscribers.end())
        subscribers[cmd] = EntityList();
    else
        for (EntityList::iterator i = subscribers[cmd].begin(); i != subscribers[cmd].end(); ++i)
            if (*i == this)
                return;
    subscribers[cmd].push_back(this);
    subscriptions.push_back(cmd);
}

void Entity::unsubscribe(std::string cmd) {
    if (subscribers.find(cmd) != subscribers.end())
        for (EntityList::iterator i = subscribers[cmd].begin(); i != subscribers[cmd].end(); ++i)
            if (*i == this)
                subscribers[cmd].erase(i);
    for (StringList::iterator i = subscriptions.begin(); i != subscriptions.end(); ++i)
        if (*i == cmd)
            subscriptions.erase(i);
}

void Entity::publish(std::string cmd, VarList args) {
    if (subscribers.find(cmd) != subscribers.end())
        for (EntityList::iterator i = subscribers[cmd].begin(); i != subscribers[cmd].end(); ++i)
            (*i)->receiveMessage(cmd,args);
}

void Entity::update(float time, float dtime) {
    for (AspectMap::iterator i = aspects.begin(); i != aspects.end(); ++i)
        i->second->onUpdate(time,dtime);
}

void Entity::loadFile(std::string filename) {
    TiXmlDocument doc(filename.c_str());
    if (doc.LoadFile()) {
        printf("loaded %s\n",filename.c_str());
        TiXmlElement* element  = doc.FirstChildElement("entity");
        if (element)
            load(element);
        else
            printf("ERROR: no root entity tag in %s\n",filename.c_str());
    } else
        printf("ERROR: couldn't open %s\n",filename.c_str());
}

void Entity::load(TiXmlElement* element) {
    for (TiXmlElement* childElement = element->FirstChildElement(); childElement; childElement = childElement->NextSiblingElement()) {
        const char* tag = childElement->Value();
        if (tag) {
            if (!strcmp(tag,"vars"))
                loadVars(childElement);
            else if (!strcmp(tag,"aspects"))
                loadAspects(childElement);
            else if (!strcmp(tag,"includes"))
                loadIncludes(childElement);
            else if (!strcmp(tag,"subscriptions"))
                loadSubscriptions(childElement);
            else
                printf("ERROR: unknown file tag %s\n",tag);
        }
    }
}

void Entity::loadVars(TiXmlElement* varsElement) {
    for (TiXmlElement* element = varsElement->FirstChildElement(); element; element = element->NextSiblingElement()) {
        const char *name = element->Attribute("name"),
                *type = element->Attribute("type"),
                *value = element->Attribute("value");
                
        if (!name) {
            printf("ERROR: var tags must have a name\n");
            return;
        }
        if (!value) {
            printf("ERROR: var tags must have a value\n");
            return;
        }
        
        setVar(name,VarUtil::stringToVar(value));
    }
}

void Entity::loadAspects(TiXmlElement* aspectsElement) {
    for (TiXmlElement* element = aspectsElement->FirstChildElement(); element; element = element->NextSiblingElement()) {
        const char *type = element->Attribute("type"),
                *name = element->Attribute("name");
                
        if (!type) {
            printf("ERROR: aspect tags must have a type\n");
            return;
        }
        
        std::string sname;
        if (name)
            sname = name;
        else
            sname = type;
        
        if (!addAspect(type,sname))
            printf("ERROR: couldn't build aspect of type %s\n",type);
    }
}

void Entity::loadIncludes(TiXmlElement* includesElement) {
    for (TiXmlElement* element = includesElement->FirstChildElement(); element; element = element->NextSiblingElement()) {
        const char *name = element->Attribute("name");
        
        if (!name)
            printf("ERROR: include tag must have a name\n");
        else {
            loadFile(name);
            includes.push_back(name);
        }
    }
}

void Entity::loadSubscriptions(TiXmlElement* subscriptionsElement) {
    for (TiXmlElement* element = subscriptionsElement->FirstChildElement(); element; element = element->NextSiblingElement()) {
        const char *cmd = element->Attribute("cmd");
        
        if (!cmd)
            printf("ERROR: subscription tag must have a cmd\n");
        else
            subscribe(cmd);
    }
}

void Entity::saveFile(std::string filename) {
    TiXmlDocument doc;
    doc.LinkEndChild(save());
    doc.SaveFile(filename.c_str());
}

TiXmlElement* Entity::save() {
    TiXmlElement* element = new TiXmlElement("entity");
    element->SetAttribute("name",getName().c_str());
    if (!vars.empty()) element->LinkEndChild(saveVars());
    if (!aspects.empty()) element->LinkEndChild(saveAspects());
    if (!includes.empty()) element->LinkEndChild(saveIncludes());
    if (!subscriptions.empty()) element->LinkEndChild(saveSubscriptions());
    return element;
}

TiXmlElement* Entity::saveVars() {
    TiXmlElement* varsElement = new TiXmlElement("vars");
    std::ostringstream sout;
    for (VarMap::iterator i = vars.begin(); i != vars.end(); ++i) {
        TiXmlElement* varElement = new TiXmlElement("var");
        varElement->SetAttribute("name",i->first.c_str());
        varElement->SetAttribute("value",VarUtil::varToString(i->second).c_str());
        varsElement->LinkEndChild(varElement);
    }
    
    return varsElement;
}

TiXmlElement* Entity::saveAspects() {
    TiXmlElement* aspectsElement = new TiXmlElement("aspects");
    
    for (AspectMap::iterator i = aspects.begin(); i != aspects.end(); ++i) {
        TiXmlElement* aspectElement = new TiXmlElement("aspect");
        aspectElement->SetAttribute("type",i->second->getType().c_str());
        aspectElement->SetAttribute("name",i->second->getName().c_str());
        aspectsElement->LinkEndChild(aspectElement);
    }
    
    return aspectsElement;
}

TiXmlElement* Entity::saveIncludes() {
    TiXmlElement* includesElement = new TiXmlElement("includes");
    
    for (StringList::iterator i = includes.begin(); i != includes.end(); ++i) {
        TiXmlElement* includeElement = new TiXmlElement("include");
        includeElement->SetAttribute("name",i->c_str());
        includesElement->LinkEndChild(includeElement);
    }
    
    return includesElement;
}

TiXmlElement* Entity::saveSubscriptions() {
    TiXmlElement* subscriptionsElement = new TiXmlElement("subscriptions");
    
    for (StringList::iterator i = subscriptions.begin(); i != subscriptions.end(); ++i) {
        TiXmlElement* subscriptionElement = new TiXmlElement("subscription");
        subscriptionElement->SetAttribute("cmd",i->c_str());
        subscriptionsElement->LinkEndChild(subscriptionElement);
    }
    
    return subscriptionsElement;
}

void Entity::log(const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer,1024,format,args);
    printf("%s: %s\n",name.c_str(),buffer);
    va_end(args);
}
