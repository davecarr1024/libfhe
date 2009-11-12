#include "PyEntity.h"
#include "PyEnv.h"
#include "PyFunc.h"

namespace fhe
{
    namespace Python
    {
        
        PyEntity::PyEntity( Entity* entity, Script* script ) :
            m_script(script),
            m_entity(entity)
        {
            assert(m_entity);
        }
        
        std::string PyEntity::getName()
        {
            return m_entity->getName();
        }
        
        std::string PyEntity::getPath()
        {
            return m_entity->getPath();
        }
        
        void PyEntity::func( boost::python::object func )
        {
            std::string name = boost::python::extract<std::string>(func.attr("__name__"));
            if ( !m_script )
            {
                throw std::runtime_error("can only add funcs to calling entity");
            }
            m_script->addFunc( new PyFunc(name,func) );
        }
        
        void PyEntity::defineClass()
        {
            boost::python::class_<PyCall>("PyCall",boost::python::no_init)
                .def("__call__",&PyCall::call)
                .def("__call__",&PyCall::callNoArg)
            ;
            
            boost::python::class_<PyEntity>("PyEntity",boost::python::no_init)
                .add_property("name",&PyEntity::getName)
                .add_property("path",&PyEntity::getPath)
                .def("hasFunc",&PyEntity::hasFunc)
                .def("hasVar",&PyEntity::hasVar)
                .def("func",&PyEntity::func)
                .def("hasAspect",&PyEntity::hasAspect)
                .def("addAspect",&PyEntity::addAspect)
                .def("removeAspect",&PyEntity::removeAspect)
                .def("getRoot",&PyEntity::getRoot)
                .def("getParent",&PyEntity::getParent)
                .def("hasChild",&PyEntity::hasChild)
                .def("getChild",&PyEntity::getChild)
                .def("buildChild",&PyEntity::buildChild)
                .def("loadChild",&PyEntity::loadChild)
                .def("removeChild",&PyEntity::removeChild)
                .def("publish",&PyEntity::publish)
                .def("__getattr__",&PyEntity::getAttr)
                .def("__setattr__",&PyEntity::setAttr)
            ;
        }
        
        bool PyEntity::hasAspect( const std::string& name )
        {
            return m_entity->hasAspect(name);
        }
        
        void PyEntity::addAspect( const std::string& name )
        {
            m_entity->buildAspect(name);
        }
        
        PyEntity PyEntity::getRoot()
        {
            return PyEntity(m_entity->getRoot().get());
        }
        
        PyEntity PyEntity::getParent()
        {
            return PyEntity(m_entity->getParent().get());
        }
        
        bool PyEntity::hasChild( const std::string& name )
        {
            return m_entity->hasChild(name);
        }
        
        PyEntity PyEntity::getChild( const std::string& name )
        {
            return PyEntity(m_entity->getChild(name).get());
        }
        
        PyEntity PyEntity::buildChild( const std::string& name )
        {
            return PyEntity(m_entity->buildChild(name).get());
        }
        
        PyEntity PyEntity::loadChild( const std::string& filename )
        {
            return PyEntity(m_entity->loadChild(filename).get());
        }
        
        bool PyEntity::hasFunc( const std::string& name )
        {
            return m_entity->hasFunc(name);
        }
        
        bool PyEntity::hasVar( const std::string& name )
        {
            return m_entity->_hasVar(name);
        }
        
        boost::python::object PyEntity::getAttr( const std::string& name )
        {
            if ( m_entity->hasFunc(name) )
            {
                return boost::python::object(PyCall(m_entity->getFunc(name)));
            }
            else if ( m_entity->_hasVar(name) )
            {
                return PyEnv::instance().convertFromVar(m_entity->_getVar(name));
            }
            else
            {
                throw std::runtime_error("entity " + getPath() + " has no vars or funcs named " + name );
            }
        }
        
        void PyEntity::setAttr( const std::string& name, boost::python::object obj )
        {
            m_entity->_setVar(name,PyEnv::instance().convertToVar(obj));
        }
        
        PyEntity::PyCall::PyCall( AbstractFunc* func ) :
            m_func(func)
        {
            assert(m_func);
        }
        
        boost::python::object PyEntity::PyCall::call( boost::python::object arg )
        {
            return PyEnv::instance().convertFromVar(m_func->call(PyEnv::instance().convertToVar(arg)));
        }
        
        boost::python::object PyEntity::PyCall::callNoArg()
        {
            return call(boost::python::object());
        }
        
        void PyEntity::removeChild( const std::string& name )
        {
            m_entity->removeChild(m_entity->getChild(name));
        }
        
        void PyEntity::removeAspect( const std::string& name )
        {
            m_entity->removeAspect(m_entity->getAspect(name));
        }
        
        void PyEntity::publish( const std::string& name, boost::python::object obj )
        {
            m_entity->_publish(name,PyEnv::instance().convertToVar(obj));
        }
    }
}
