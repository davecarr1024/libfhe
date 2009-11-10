#include "PyEntity.h"
#include "PyEnv.h"
#include "PyFunc.h"

namespace fhe
{
    namespace Python
    {
        
        PyEntity::PyEntity( Script* script, Entity* entity ) :
            m_script(script),
            m_entity(entity)
        {
            assert(m_script);
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
                .def("__getattr__",&PyEntity::getAttr)
                .def("__setattr__",&PyEntity::setAttr)
            ;
        }
        
        bool PyEntity::hasFunc( const std::string& name )
        {
            return m_entity->hasFunc(name);
        }
        
        bool PyEntity::hasVar( const std::string& name )
        {
            return m_entity->hasVarName(name);
        }
        
        boost::python::object PyEntity::getAttr( const std::string& name )
        {
            if ( m_entity->hasFunc(name) )
            {
                return boost::python::object(PyCall(m_entity->getFunc(name)));
            }
            else if ( m_entity->hasVarName(name) )
            {
                return PyEnv::instance().convertFromVar(m_entity->getRawVar(name));
            }
            else
            {
                throw std::runtime_error("entity " + getPath() + " has no vars or funcs named " + name );
            }
        }
        
        void PyEntity::setAttr( const std::string& name, boost::python::object obj )
        {
            m_entity->setRawVar(name,PyEnv::instance().convertToVar(obj));
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
    }
}
