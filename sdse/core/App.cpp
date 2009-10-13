#include "App.h"

#include <ctime>
#include <cstring>
#include <signal.h>

#include "tinyxml/tinyxml.h"
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace sdse;

bool App::shutdownAll = false;

App::App() : shutdownRequested(false) {
}

App::~App() {
    for (EntityMap::iterator i = entities.begin(); i != entities.end(); ++i) {
        i->second->detach();
        delete i->second;
    }
}

void App::addEntity(Entity* entity) {
    if (!hasEntity(entity->getName())) {
        entities[entity->getName()] = entity;
        entity->attach(this);
    }
}

void App::removeEntity(Entity* entity) {
    if (hasEntity(entity->getName())) {
        entities.erase(entity->getName());
        entity->detach();
    }
}

void App::deleteEntity(std::string name) {
    Entity* entity = getEntity(name);
    if (entity) {
        removeEntity(entity);
        delete entity;
    }
}

bool App::hasEntity(std::string name) {
    return entities.find(name) != entities.end();
}

Entity* App::getEntity(std::string name) {
    EntityMap::iterator i = entities.find(name);
    if (i != entities.end())
        return i->second;
    else
        return 0;
}

void shutdownSignal(int sig) {
    printf("ctrl-c\n");
    App::shutdownAll = true;
}

void App::run() {
    signal(SIGABRT,&shutdownSignal);
    signal(SIGTERM,&shutdownSignal);
    signal(SIGINT,&shutdownSignal);
    
    int numFrames = 0;
    float time = getTime(), lastTime = time, dtime, startTime = time;
    while (!shutdownRequested && !shutdownAll) {// && time - startTime < 0.1) {
        numFrames++;
        
        time = getTime() - startTime;
        dtime = time - lastTime;
        lastTime = time;
        
        for (EntityMap::iterator i = entities.begin(); i != entities.end(); ++i)
            i->second->update(time,dtime);
        
        for (AppListenerList::iterator i = listeners.begin(); i != listeners.end(); ++i)
            (*i)->update(time,dtime);
    }
    
    for (AppListenerList::iterator i = listeners.begin(); i != listeners.end(); ++i)
        (*i)->shutdown();
    
    float totalTime = getTime() - startTime;
    printf("%d frames in %f secs = %f fps\n",numFrames,totalTime,float(numFrames)/totalTime);
}

void App::shutdown() {
    shutdownRequested = true;
}

float App::getTime() {
    return float(boost::posix_time::microsec_clock::universal_time()
            .time_of_day().total_nanoseconds()) / 1000000000.0;
//     return float(clock()) / float(CLOCKS_PER_SEC);
}

void App::loadFile(std::string filename) {
    TiXmlDocument doc(filename.c_str());
    if (doc.LoadFile()) {
        for (TiXmlElement* element = doc.FirstChildElement(); element; element = element->NextSiblingElement()) {
            const char* tag = element->Value();
            if (tag) {
                if (!strcmp(tag,"entity")) {
                    const char* entityName = element->Attribute("name");
                    std::string name;
                    if (entityName)
                        name = entityName;
                    else
                        name = "entity";
                    Entity* ent = new Entity(name);
                    addEntity(ent);
                    ent->load(element);
                } else
                    printf("ERROR: unknown tag %s in file %s\n",tag,filename.c_str());
            }
        }
    } else
        printf("ERROR: unable to load file %s\n",filename.c_str());
}

void App::saveFile(std::string filename) {
    TiXmlDocument doc;
    
    for (EntityMap::iterator i = entities.begin(); i != entities.end(); ++i)
        doc.LinkEndChild(i->second->save());
    
    doc.SaveFile(filename.c_str());
}

void App::addListener(AppListener* listener) {
    bool found = false;
    for (AppListenerList::iterator i = listeners.begin(); i != listeners.end(); ++i)
        if (*i == listener)
            found = true;
    if (!found) {
        listeners.push_back(listener);
        listener->attach(this);
    }
}

void App::removeListener(AppListener* listener) {
    for (AppListenerList::iterator i = listeners.begin(); i != listeners.end(); ++i)
        if (*i == listener) {
            listeners.erase(i);
            (*i)->detach();
        }
}
