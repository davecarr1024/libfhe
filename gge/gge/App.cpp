#include "App.h"
#include "FileSystem.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>

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
    
    void App::publish( const std::string& cmd, const Var& arg )
    {
        std::string msg = "msg_" + cmd;
        for ( EntityMap::iterator i = m_entities.begin(); i != m_entities.end(); ++i )
        {
            i->second->callAll(msg,arg);
        }
    }
    
    void App::load( const std::string& filename )
    {
        TiXmlDocument doc;
        if ( doc.LoadFile( FileSystem::instance().getFile(filename).c_str() ) )
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
    
    float App::getTime()
    {                   
        return float(boost::posix_time::microsec_clock::universal_time()
            .time_of_day().total_nanoseconds()) / 1000000000.0;
    }
    
    void App::shutdown()
    {
        m_shutdown = true;
    }
    
    void App::run( float maxTime )
    {
        m_shutdown = false;
        float time = getTime(), lastTime = time, startTime = time, dtime;
        int numFrames = 0;
        
        while ( (maxTime < 0 || time - startTime < maxTime) && !m_shutdown )
        {
            numFrames++;
            time = getTime();
            dtime = lastTime - time;
            lastTime = time;
            
            VarMap args;
            args.setVar<float>("time",time - startTime);
            args.setVar<float>("dtime",dtime);
            publish("update",Var::build<VarMap>(args));
        }
        
        printf("fps %f\n",float(numFrames)/(getTime()-startTime));
    }
    
}
