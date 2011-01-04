#ifndef FHE_TEXT_SCREEN_H
#define FHE_TEXT_SCREEN_H

#include <fhe/core/Node.h>
#include <fhe/sim/IUpdate.h>
#include <fhe/text/RenderContext.h>

namespace fhe
{
    namespace text
    {
        class Screen : public Node, public sim::IUpdate
        {
            private:
                RenderContext m_rc;
                
            public:
                Screen();
                virtual ~Screen();
                
                void update( double time, double dtime );
        };
    }
}

#endif
