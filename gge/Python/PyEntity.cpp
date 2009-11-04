#include "PyEntity.h"
#include "Env.h"

namespace gge
{
    namespace Python
    {
        PyEntity::PyEntity( Entity* entity ) :
            m_entity(entity)
        {
        }
        
        void PyEntity::defineClass()
        {
            boost::python::class_<PyEntity>("PyEntity",boost::python::no_init)
                .add_property("name",&PyEntity::getName)
                .add_property("app",&PyEntity::getApp)
                .def("__getattr__",&PyEntity::getAttr)
                .def("__setattr__",&PyEntity::setAttr)
                .def("hasAspect",&PyEntity::hasAspect)
                .def("addAspect",&PyEntity::addAspect)
                .def("removeAspect",&PyEntity::removeAspect)
            ;
            
            boost::python::class_<FuncCall>("FuncCall",boost::python::no_init)
                .def("__call__",&FuncCall::call)
                .def("__call__",&FuncCall::callNoArg)
            ;
        }
        
        std::string PyEntity::getName()
        {
            return m_entity->getName();
        }
        
        boost::python::object PyEntity::getAttr( const std::string& name )
        {
            if ( m_entity->hasFunc(name) )
            {
                return boost::python::object(FuncCall(m_entity->getFunc(name)));
            }
            else if ( m_entity->hasVarName(name) )
            {
                return Env::instance().convertFromVar(m_entity->getRawVar(name));
            }
            
            throw std::runtime_error("entity " + m_entity->getName() + " has no vars or funcs named " + name);
        }
        
        void PyEntity::setAttr( const std::string& name, boost::python::object val )
        {
            m_entity->setRawVar(name,Env::instance().convertToVar(val));
        }
        
        PyEntity::FuncCall::FuncCall( AbstractFunc* func ) :
            m_func(func)
        {
            assert(m_func);
        }
        
        boost::python::object PyEntity::FuncCall::callNoArg()
        {
            return call(boost::python::object());
        }
        
        boost::python::object PyEntity::FuncCall::call( boost::python::object arg )
        {
            return Env::instance().convertFromVar( m_func->call( Env::instance().convertToVar(arg) ) );
        }
        
        bool PyEntity::hasAspect( const std::string& name )
        {
            return m_entity->hasAspect(name);
        }
        
        void PyEntity::addAspect( const std::string& name )
        {
            m_entity->buildAspect(name);
        }
        
        void PyEntity::removeAspect( const std::string& name )
        {
            m_entity->removeAspect(m_entity->getAspect(name));
        }
        
        PyApp PyEntity::getApp()
        {
            return PyApp(m_entity->getApp());
        }
        
    }
}
