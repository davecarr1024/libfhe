#ifndef FHE_SIM_H
#define FHE_SIM_H

#include <fhe/core/Node.h>

namespace fhe
{
    namespace sim
    {
    
        class Sim : public Node
        {
            private:
                bool m_shutdown;
                
            public:
                Sim();
                virtual ~Sim();
                
                double time();
                void run();
                void shutdown();
        };

    }
}

#endif
