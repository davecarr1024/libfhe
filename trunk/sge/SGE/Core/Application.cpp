#include "Application.h"
#include "NodeFactory.h"

#include <Poco/Timespan.h>

namespace SGE
{
    
    Application::Application() :
        m_root(NodeFactory::instance().buildNode("Node","Root")),
        m_shutdown(false)
    {
    }
    
    Application::~Application()
    {
    }
    
    Application& Application::instance()
    {
        static Application app;
        return app;
    }
    
    NodePtr Application::getRoot()
    {
        return m_root;
    }
    
    void Application::shutdown()
    {
        m_shutdown = true;
    }
    
    float Application::time()
    {
        static Poco::Timestamp timestamp;
        return float(timestamp.elapsed()) / 1000000.0f;
    }
    
    float Application::run(float timeToRun)
    {
        Poco::Timestamp timeStamp;
        float startTime = time(), lastTime = startTime, currentTime, dtime;
        int numFrames = 0;
        
        while (!m_shutdown && (timeToRun < 0 || lastTime - startTime < timeToRun))
        {
            numFrames++;
            currentTime = time();
            dtime = currentTime - lastTime;
            lastTime = currentTime;
        }
        
        return float(numFrames) / (time() - startTime);
    }
}
