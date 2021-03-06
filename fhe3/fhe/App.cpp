#include "App.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace fhe
{
    FHE_ASPECT(App);
    
    App::App()
    {
    }
    
    float App::getTime()
    {
        return float(boost::posix_time::microsec_clock::universal_time()
            .time_of_day().total_nanoseconds()) / 1000000000.0;
    }
    
    void App::run( float maxTime )
    {
        int numFrames = 0;
        float startTime = getTime(), time = 0, lastTime = time, dtime;
        while ( !getEntity()->getVar<bool>("shutdown",false) && (maxTime < 0 || time < maxTime) )
        {
            ++numFrames;
            time = getTime() - startTime;
            dtime = time - lastTime;
            lastTime = time;

            getEntity()->setVar<float>("time",time);
            VarMap args;
            args.setVar("time",time);
            args.setVar("dtime",time);
            getEntity()->publish("update",args);
        }
        float totalTime = getTime() - startTime;
        log("fps %f", float(numFrames) / totalTime );
    }
}
