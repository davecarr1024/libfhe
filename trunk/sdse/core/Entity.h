#ifndef ENTITY_H
#define ENTITY_H

#include "Var.h"
#include "Aspect.h"

#include "tinyxml/tinyxml.h"

#include <string>
#include <map>

namespace sdse {

    class Entity;
    class App;
    
    typedef std::map<std::string, Entity*> EntityMap;
    typedef std::vector<Entity*> EntityList;
    typedef std::map<std::string, EntityList> EntityListMap;
    
    class Entity {
        private:
            static EntityMap entities;
            static EntityListMap subscribers;

            VarMap vars;
            AspectMap aspects;
            std::string name;
            App* app;
            
            StringList includes, subscriptions;
            
            Aspect* aspectFactory(std::string aspectType, std::string aspectName);
            
            void loadVars(TiXmlElement* element);
            void loadAspects(TiXmlElement* element);
            void loadIncludes(TiXmlElement* element);
            void loadSubscriptions(TiXmlElement* element);

            TiXmlElement* saveVars();
            TiXmlElement* saveAspects();
            TiXmlElement* saveIncludes();
            TiXmlElement* saveSubscriptions();
            
        public:
            Entity(std::string _name);
            ~Entity();

            App* getApp();
            std::string getName();
            
            void attach(App* _app);
            void detach();
            
            void addAspect(Aspect* aspect);
            void removeAspect(Aspect* aspect);
            void deleteAspect(std::string name);
            bool hasAspect(std::string name);
            Aspect* getAspect(std::string name);
            Aspect* addAspect(std::string aspectType, std::string aspectName);
            Aspect* getFirstAspectOfArchetype(std::string archetype);
            AspectList getAllAspectsOfArchetype(std::string archetype);
            
            bool hasVar(std::string varName);
            Var getVar(std::string varName);
            Var getVar(std::string varName, Var defaultVal);
            void setVar(std::string varName, Var val);
            template <typename T> bool queryVar(std::string varName, T& val) {
                Var var = getVar(varName);
                if (boost::any_cast<T>(&var)) {
                    val = boost::any_cast<T>(var);
                    return true;
                } else
                    return false;
            }
            template <typename T> T defaultVar(std::string varName, T defaultVal) {
                Var var = getVar(varName);
                if (boost::any_cast<T>(&var))
                    return boost::any_cast<T>(var);
                else
                    return defaultVal;
            }
            
            void receiveMessage(std::string cmd, VarList args);
            void subscribe(std::string cmd);
            void unsubscribe(std::string cmd);
            static void publish(std::string cmd, VarList args);
            
            void update(float time, float dtime);
            
            void loadFile(std::string filename);
            void load(TiXmlElement* element);
            
            void saveFile(std::string filename);
            TiXmlElement* save();
            
            void log(const char* format, ...);
    };
    
}

#endif
