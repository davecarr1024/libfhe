#include <boost/python.hpp>
#include "Var.h"
#include "VarMap.h"
#include "Entity.h"
#include "Aspect.h"

namespace fhe
{
    template <class T>
    class FromPyConverter
    {
        public:
            T operator()(boost::python::object obj )
            {
                return boost::python::extract<T>(obj);
            }
    };
    
    template <>
    class FromPyConverter<boost::python::object>
    {
        public:
            boost::python::object operator()( boost::python::object obj )
            {
                return obj;
            }
    };
    
    template <>
    class FromPyConverter<Var>
    {
        public:
            Var operator()( boost::python::object obj )
            {
                return Var::fromPy(obj);
            }
    };
    
    template <>
    class FromPyConverter<VarMap>
    {
        public:
            VarMap operator()( boost::python::object obj )
            {
                return VarMap::fromPy(obj);
            }
    };
    
    template <>
    class FromPyConverter<EntityPtr>
    {
        public:
            EntityPtr operator()( boost::python::object obj )
            {
                return EntityPtr(boost::python::extract<Entity*>(obj)(),true);
            }
    };
    
    template <>
    class FromPyConverter<AspectPtr>
    {
        public:
            AspectPtr operator()( boost::python::object obj )
            {
                return AspectPtr(boost::python::extract<Aspect*>(obj)(),true);
            }
    };
    
    template <class T>
    class ToPyConverter
    {
        public:
            boost::python::object operator()( const T& obj )
            {
                return boost::python::object(obj);
            }
    };
    
    template <>
    class ToPyConverter<Var>
    {
        public:
            boost::python::object operator()( const Var& obj )
            {
                return const_cast<Var&>(obj).toPy();
            }
    };
    
    template <>
    class ToPyConverter<VarMap>
    {
        public:
            boost::python::object operator()( const VarMap& obj )
            {
                return const_cast<VarMap&>(obj).toPy();
            }
    };
    
    template <>
    class ToPyConverter<EntityPtr>
    {
        public:
            boost::python::object operator()( const EntityPtr& obj )
            {
                return boost::python::object(boost::python::ptr(obj.get()));
            }
    };
    
    template <>
    class ToPyConverter<AspectPtr>
    {
        public:
            boost::python::object operator()( const AspectPtr& obj )
            {
                return boost::python::object(boost::python::ptr(obj.get()));
            }
    };
}
