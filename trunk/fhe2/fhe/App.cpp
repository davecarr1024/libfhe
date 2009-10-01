#include "App.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace fhe
{
    FHE_NODE_IMPL(App);
    
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
        while ( !getVar<bool>("shutdown",false) && (maxTime < 0 || time < maxTime) )
        {
            ++numFrames;
            time = getTime() - startTime;
            dtime = time - lastTime;
            lastTime = time;

            setVar<float>("time",time);
            VarMap args;
            args.setVar("time",time);
            args.setVar("dtime",time);
            publish("update",args);
            
            #ifdef FHE_THREAD
            Node::threadPool.joinAll();
            #endif
        }
        float totalTime = getTime() - startTime;
        log("fps %f", float(numFrames) / totalTime );
    }
}
