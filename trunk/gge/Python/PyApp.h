#ifndef PYTHON_APP_H
#define PYTHON_APP_H

#include <gge/App.h>
#include <boost/python.hpp>

namespace gge
{
    namespace Python
    {
        
        class PyApp
        {
            private:
                App* m_app;
                
            public:
                PyApp( App* app );
                
                bool hasEntity( const std::string& name );
                
                boost::python::object getEntity( const std::string& name );
                
                boost::python::object addEntity( const std::string& name );
                
                void removeEntity( const std::string& name );
                
                void shutdown();
                
                static void defineClass();
        };
        
    }
}

#endif
