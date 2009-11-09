#include <boost/date_time/posix_time/posix_time_types.hpp> 
#include "App.h"

namespace fhe
{
    
    FHE_ASPECT(App,Aspect);
    
    FHE_FUNC_IMPL(App,run)
    {
        float startTime = get_time(Var()).get<float>(), 
            time = 0, 
            maxTime = getEntity()->getVar<float>("maxTime",-1);
        int frames = 0;
        
        while ( !getEntity()->getVar<bool>("shutdown",false) && (maxTime < 0 || time < maxTime ) )
        {
            ++frames;
            time = get_time(Var()).get<float>() - startTime;
            getEntity()->publish("update",Var::build<float>(time));
        }
        
        log("fps %f",float(frames)/time);
        
        return Var();
    }
    
    FHE_FUNC_IMPL(App,shutdown)
    {
        getEntity()->setVar<bool>("shutdown",true);
        return Var();
    }
    
    FHE_FUNC_IMPL(App,get_time)
    {
        return Var::build<float>((boost::posix_time::microsec_clock::universal_time()
            .time_of_day().total_nanoseconds()) / 1000000000.0);
    }
    
}
