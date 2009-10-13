#include "Aspect.h"
#include "Entity.h"

#include <sstream>
#include <cstdarg>

using namespace sdse;

AspectMap Aspect::aspects;

Aspect::Aspect(std::string _archetype, std::string _type, std::string _name) :
    entity(0),
    archetype(_archetype),
    type(_type) {
    if (aspects.find(_name) == aspects.end())
        name = _name;
    else {
        std::ostringstream sout;
        int i = 2;
        do {
            sout << _name << "_" << i;
            name = sout.str();
            sout.str("");
            i++;
        } while (aspects.find(name) != aspects.end());
    }
    aspects[name] = this;
}

Aspect::~Aspect() {
    aspects.erase(name);
    detach();
}

void Aspect::attach(Entity* _entity) {
    if (entity != _entity) {
        detach();
        entity = _entity;
        entity->addAspect(this);
        log("attach");
        onAttach(_entity);
        log("/attach");
    }
}

void Aspect::detach() {
    if (entity) {
        Entity* _entity = entity;
        entity = 0;
        _entity->removeAspect(this);
        log("detach");
        onDetach(_entity);
        log("/detach");
    }
}

Entity* Aspect::getEntity() {
    return entity;
}

std::string Aspect::getName() {
    return name;
}

std::string Aspect::getType() {
    return type;
}

std::string Aspect::getArchetype() {
    return archetype;
}

bool Aspect::hasAspect(std::string name) {
    return aspects.find(name) != aspects.end();
}

Aspect* Aspect::getAspect(std::string name) {
    AspectMap::iterator i = aspects.find(name);
    if (i != aspects.end())
        return i->second;
    else
        return 0;
}

void Aspect::log(const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer,1024,format,args);
    if (entity)
        entity->log("%s: %s",name.c_str(),buffer);
    else
        printf("(no ent): %s: %s\n",name.c_str(),buffer);
    va_end(args);
}
