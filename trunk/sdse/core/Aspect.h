#ifndef ASPECT_H
#define ASPECT_H

#include "Var.h"

#include <string>
#include <map>
#include <vector>

namespace sdse {

    class Aspect;
    class Entity;
    
    typedef std::map<std::string, Aspect*> AspectMap;
    typedef std::vector<Aspect*> AspectList;
    
    class Aspect {
        private:
            static AspectMap aspects;
            
        protected:
            Entity* entity;
            std::string archetype, type, name;

        public:
            Aspect(std::string _archetype, std::string _type, std::string _name);
            virtual ~Aspect();
            
            Entity* getEntity();
            std::string getName();
            std::string getType();
            std::string getArchetype();
            
            void attach(Entity* _entity);
            void detach();
            
            void log(const char* format, ...);
            
            virtual void onAttach(Entity* attachedTo) {}
            virtual void onDetach(Entity* detachedFrom) {}
            virtual void onSetVar(std::string varName, Var val) {}
            virtual void onReceiveMessage(std::string cmd, VarList args) {}
            virtual void onUpdate(float time, float dtime) {}
            
            static bool hasAspect(std::string name);
            static Aspect* getAspect(std::string name);
    };
    
}

#endif
