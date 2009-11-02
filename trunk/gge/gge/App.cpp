#include "App.h"

namespace gge
{
    
    App::App()
    {
    }
    
    void App::addEntity( EntityPtr entity )
    {
        if ( entity && !hasEntity(entity->getName()) )
        {
            m_entities[entity->getName()] = entity;
            entity->attachToApp(this);
        }
    }
    
    void App::removeEntity( EntityPtr entity )
    {
        if ( entity && hasEntity(entity->getName()) )
        {
            m_entities.erase(entity->getName());
            entity->detachFromApp();
        }
    }
    
    bool App::hasEntity( const std::string& name )
    {
        return m_entities.find(name) != m_entities.end();
    }
    
    EntityPtr App::getEntity( const std::string& name )
    {
        return hasEntity(name) ? m_entities[name] : 0;
    }
    
    EntityPtr App::buildEntity( const std::string& name )
    {
        EntityPtr entity( new Entity(name) );
        addEntity(entity);
        return entity;
    }
    
    void App::publish( const std::string& cmd, const VarMap& args )
    {
        std::string msg = "msg_" + cmd;
        for ( EntityMap::iterator i = m_entities.begin(); i != m_entities.end(); ++i )
        {
            i->second->callAll<void,VarMap>(msg,args);
        }
    }
    
    void App::load( const std::string& filename )
    {
        TiXmlDocument doc;
        if ( doc.LoadFile( filename.c_str() ) )
        {
            loadData( &doc );
        }
        else
        {
            throw std::runtime_error("unable to load app file " + filename);
        }
    }
    
    void App::loadData( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement("entity").ToElement(); e; e = e->NextSiblingElement("entity") )
        {
            const char* name = e->Attribute("name");
            assert(name);
            buildEntity(name)->loadData(e);
        }
    }
    
    boost::python::object App::pyGetEntity( const std::string& name )
    {
        EntityPtr entity = getEntity(name);
        return entity ? entity->toPy() : boost::python::object();
    }
    
    boost::python::object App::pyBuildEntity( const std::string& name )
    {
        return buildEntity(name)->toPy();
    }
    
    void App::pyPublish( const std::string& cmd, boost::python::dict args )
    {
        publish(cmd,VarMap::fromPy(args));
    }
    
    boost::python::object App::defineClass()
    {
        return boost::python::class_<App,boost::noncopyable>("App",boost::python::no_init)
            .def("hasEntity",&App::hasEntity)
            .def("getEntity",&App::pyGetEntity)
            .def("buildEntity",&App::pyBuildEntity)
            .def("publish",&App::pyPublish)
        ;
    }
    
    boost::python::object App::toPy()
    {
        return boost::python::object(boost::python::ptr(this));
    }
    
}
