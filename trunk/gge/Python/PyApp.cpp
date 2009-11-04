#include "PyApp.h"
#include "PyEntity.h"

namespace gge
{
    namespace Python
    {
        
        PyApp::PyApp( App* app ) :
            m_app(app)
        {
            assert(m_app);
        }
        
        bool PyApp::hasEntity( const std::string& name )
        {
            return m_app->hasEntity(name);
        }
        
        boost::python::object PyApp::getEntity( const std::string& name )
        {
            EntityPtr entity = m_app->getEntity(name);
            return entity ? boost::python::object(PyEntity(entity.get())) : boost::python::object();
        }
        
        boost::python::object PyApp::addEntity( const std::string& name )
        {
            return boost::python::object(PyEntity(m_app->buildEntity(name).get()));
        }
        
        void PyApp::removeEntity( const std::string& name )
        {
            m_app->removeEntity(m_app->getEntity(name));
        }
        
        void PyApp::shutdown()
        {
            m_app->shutdown();
        }
        
        void PyApp::defineClass()
        {
            boost::python::class_<PyApp>("PyApp",boost::python::no_init)
                .def("hasEntity",&PyApp::hasEntity)
                .def("getEntity",&PyApp::getEntity)
                .def("addEntity",&PyApp::addEntity)
                .def("removeEntity",&PyApp::removeEntity)
                .def("shutdown",&PyApp::shutdown)
            ;
        }
        
    }
}
