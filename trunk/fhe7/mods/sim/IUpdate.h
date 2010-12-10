#ifndef FHE_SIM_IUPDATE_H
#define FHE_SIM_IUPDATE_H

namespace fhe
{
    namespace sim
    {
    
        class IUpdate
        {
            public:
                virtual void update( double time, double dtime ) = 0;
        };
        
    }
}

#endif
