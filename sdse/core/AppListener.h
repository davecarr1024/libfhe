#ifndef APPLISTENER_H
#define APPLISTENER_H

#include <vector>

namespace sdse {
    
    class App;
    
    class AppListener {
        protected:
            App* app;
        
        public:
            AppListener(App* _app);
            AppListener();
            
            void attach(App* _app);
            void detach();
            
            virtual void update(float time, float dtime) {}
            virtual void shutdown() {}
    };
    
    typedef std::vector<AppListener*> AppListenerList;
    
}

#endif
