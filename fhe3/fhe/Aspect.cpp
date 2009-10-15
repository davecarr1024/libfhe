#include "Aspect.h"
#include "Entity.h"
#include "FileSystem.h"

#include "math/Vec2.h"
#include "math/Vec3.h"
#include "math/Rot.h"
#include "math/Quat.h"

#include <stdexcept>
#include <cstdarg>
#include <cstdio>

namespace fhe
{
    
    FHE_ASPECT(Aspect);
    
    FHE_TO_CONVERTER(AspectPtr,boost::python::object(boost::python::ptr(obj.get())));
    FHE_FROM_CONVERTER(AspectPtr,obj == boost::python::object() ? 0 : AspectPtr(boost::python::extract<Aspect*>(obj)(),true));
    
    bool Aspect::m_pythonInitialized = false;
    boost::python::object Aspect::m_mainModule;
    boost::python::object Aspect::m_mainNamespace;
    
    Aspect::Aspect() :
        m_entity(0)
    {
        addFunc("getName",&Aspect::getName,this);
        addFunc("getPath",&Aspect::getPath,this);
        addFunc("getEntity",&Aspect::getEntity,this);
    }
    
    void Aspect::init( const std::string& name )
    {
        m_name = name;
    }
    
    std::string Aspect::getName()
    {
        return m_name;
    }
    
    std::string Aspect::getPath()
    {
        return m_entity ? m_entity->getPath() + "." + m_name : "<None>:" + m_name;
    }
    
    boost::python::object Aspect::getAttr( const std::string& name )
    {
        if ( hasFuncName( name ) )
        {
            return boost::python::object(pyGetFunc(name));
        }
        else
        {
            error("name error: %s", name.c_str());
        }
    }
    
    void Aspect::runScript( const std::string& filename )
    {
        runScript(filename,defaultNamespace());
    }
    
    void Aspect::runScript( const std::string& filename, boost::python::dict ns )
    {
        try
        {
            initializePython();
            
            boost::python::exec_file(FileSystem::instance().getFile(filename).c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            throw;
        }
    }
    
    boost::python::object Aspect::tryEvalScript( const std::string& s, boost::python::dict ns )
    {
        try 
        {
            initializePython();
            
            return Var::fromPy(boost::python::eval(s.c_str(),ns,ns)).toPy();
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Clear();
            return boost::python::str(s);
        }
    }
    
    void Aspect::execScript( const std::string& s, boost::python::dict ns )
    {
        try
        {
            initializePython();
            
            boost::python::exec(s.c_str(),ns,ns);
        }
        catch ( boost::python::error_already_set const& )
        {
            PyErr_Print();
            throw;
        }
    }
    
    boost::python::dict Aspect::defaultNamespace() 
    {
        boost::python::dict ns = emptyNamespace();
        ns["self"] = toPy();
        ns["entity"] = boost::python::object(boost::python::ptr(m_entity));
        return ns;
    }
    
    boost::python::dict Aspect::emptyNamespace()
    {
        initializePython();
        boost::python::dict ns;
        ns.update(m_mainNamespace);
        return ns;
    }
    
    boost::python::object Aspect::toPy()
    {
        return boost::python::object(boost::python::ptr(this));
    }
    
    void Aspect::initializePython()
    {
        if ( !m_pythonInitialized )
        {
            m_pythonInitialized = true;
            
            Py_Initialize();
            
            m_mainModule = boost::python::import("__main__");
            m_mainNamespace = boost::python::dict(m_mainModule.attr("__dict__"));
            
            defineClass();
            PyCall::defineClass();
            FuncClosure::defineClass();
            Entity::defineClass();
            
            m_mainNamespace["Vec2"] = Vec2::defineClass();
            m_mainNamespace["Vec3"] = Vec3::defineClass();
            m_mainNamespace["Rot"] = Rot::defineClass();
            m_mainNamespace["Quat"] = Quat::defineClass();
        }
    }
    
    boost::python::object Aspect::defineClass()
    {
        return boost::python::class_<Aspect, boost::noncopyable>("Aspect",boost::python::no_init)
            .def("__getattr__",&Aspect::getAttr)
            .def("func",&Aspect::func)
            .def("log",&Aspect::pyLog)
        ;
    }
    
    FuncMap::FuncClosure Aspect::func( boost::python::object tret, boost::python::object targ )
    {
        return FuncClosure(this,tret,targ);
    }
    
    void Aspect::setEntity( EntityPtr entity )
    {
        m_entity = entity.get();
    }
    
    EntityPtr Aspect::getEntity()
    {
        return m_entity ? EntityPtr(m_entity,true) : 0;
    }
    
    EntityPtr Aspect::getParentEntity()
    {
        EntityPtr entity(getEntity());
        return entity ? entity->getParent() : 0;
    }
    
    void Aspect::load( TiXmlHandle h )
    {
        for ( TiXmlElement* e = h.FirstChildElement().ToElement(); e; e = e->NextSiblingElement() )
        {
            std::string tag(e->Value());
            
            if ( tag == "script" )
            {
                runScript(e->GetText(),defaultNamespace());
            }
            else
            {
                throw std::runtime_error("unknown aspect file tag " + tag );
            }
        }
    }

    void Aspect::log( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        printf("%s: %s\n", getPath().c_str(), buffer);
    }

    void Aspect::error( const char* format, ... )
    {
        char buffer[1024];
        va_list ap;
        va_start(ap,format);
        vsnprintf( buffer, 1024, format, ap );
        va_end(ap);
        throw std::runtime_error( getPath() + ": ERROR: " + buffer );
    }
    
    void Aspect::pyLog( const std::string& s )
    {
        log(s.c_str());
    }
}
