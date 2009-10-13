#ifndef APP_H
#define APP_H

#include "Entity.h"
#include "AppListener.h"

namespace sdse {
    
    class App {
        private:
            EntityMap entities;
            bool shutdownRequested;
            AppListenerList listeners;
            
        public:
            static bool shutdownAll;

            App();
            ~App();
            
            void addEntity(Entity* entity);
            void removeEntity(Entity* entity);
            bool hasEntity(std::string name);
            Entity* getEntity(std::string name);
            void deleteEntity(std::string name);
            
            void run();
            void shutdown();
            float getTime();
            
            void loadFile(std::string filename);
            void saveFile(std::string filename);
            
            void addListener(AppListener* listener);
            void removeListener(AppListener* listener);
    };
    
}

#endif
