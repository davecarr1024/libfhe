#ifndef APPLICATION_H
#define APPLICATION_H

#include "Node.h"

namespace SGE
{

    class Application
    {
        private:
            NodePtr m_root;
            
            bool m_shutdown;
            
            Application();

        public:
            virtual ~Application();

            static Application& instance();
            
            NodePtr getRoot();
            
            float time();
            float run(float timeToRun = -1);
            void shutdown();
    };

}

#endif
