#include <sim/Sim.h>
#include <sim/IUpdate.h>
#include <boost/date_time/posix_time/posix_time_types.hpp> 

namespace fhe
{
    namespace sim
    {
        
        FHE_NODE( Sim )
        FHE_FUNC( Sim, time )
        FHE_FUNC( Sim, run )
        FHE_FUNC( Sim, shutdown )
        
        Sim::Sim() :
            m_shutdown( false )
        {
        }
        
        Sim::~Sim()
        {
        }
        
        void Sim::shutdown()
        {
            m_shutdown = true;
        }
        
        double Sim::time()
        {
            return boost::posix_time::microsec_clock::universal_time().time_of_day().total_nanoseconds() / 1000000000.0;
        }
        
        void Sim::run()
        {
            double start = time(), now = start, lastTime = now, dtime;
            while ( !m_shutdown )
            {
                now = time();
                dtime = now - lastTime;
                lastTime = now;
                publish( &IUpdate::update, now - start, dtime );
            }
        }
    }
}
