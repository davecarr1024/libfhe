#include "App.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace fhe
{
    NODE_IMPL(App);
    
    App::App( const std::string& type, const std::string& name ) :
        Node( type, name )
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
        float startTime = getTime(), time = 0;
        while ( !getVar<bool>("shutdown",false) && (maxTime < 0 || time < maxTime) )
        {
            ++numFrames;
            time = getTime() - startTime;
            setVar<float>("time",time);
            publish<float>("update",time);
        }
        float totalTime = getTime() - startTime;
        log("fps %f", float(numFrames) / totalTime );
    }
}
