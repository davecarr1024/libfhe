#include "Var.h"
#include "VarMap.h"
#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Vec3.h"
#include "math/Quat.h"

namespace fhe
{
    Var::Var() :
        m_data(0)
    {
    }
    
    Var::Var( const Var& var ) :
        m_data(var.m_data ? var.m_data->clone() : 0)
    {
    }
    
    Var::Var( boost::python::object obj ) :
        m_data(0)
    {
        *this = Var::fromPy(obj);
    }
    
    Var::~Var()
    {
        clear();
    }
    
    Var& Var::operator=( const Var& var )
    {
        clear();
        m_data = var.m_data ? var.m_data->clone() : 0;
    }
    
    void Var::clear()
    {
        if ( m_data )
        {
            delete m_data;
            m_data = 0;
        }
    }
    
    boost::python::object Var::toPy()
    {
        if ( is<bool>() )
        {
            return boost::python::object(get<bool>());
        }
        else if ( is<int>() )
        {
            return boost::python::object(get<int>());
        }
        else if ( is<float>() )
        {
            return boost::python::object(get<float>());
        }
        else if ( is<std::string>() )
        {
            return boost::python::object(get<std::string>());
        }
        else if ( is<VarMap>() )
        {
            return boost::python::object(get<VarMap>());
        }
        else if ( is<Var>() )
        {
            return boost::python::object(get<Var>());
        }
        else if ( is<Vec2>() )
        {
            return boost::python::object(get<Vec2>());
        }
        else if ( is<Rot>() )
        {
            return boost::python::object(get<Rot>());
        }
        else if ( is<Vec3>() )
        {
            return boost::python::object(get<Vec3>());
        }
        else if ( is<Quat>() )
        {
            return boost::python::object(get<Quat>());
        }
        else
        {
            return boost::python::object();
        }
    }
    
    Var Var::fromPy( boost::python::object obj )
    {
        Var var;
        
        std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));

        if ( type == "bool" )
        {
            var.set(boost::python::extract<bool>(obj)());
        }
        else if ( type == "int" )
        {
            var.set(boost::python::extract<int>(obj)());
        }
        else if ( type == "float" )
        {
            var.set(boost::python::extract<float>(obj)());
        }
        else if ( type == "str" )
        {
            var.set(boost::python::extract<std::string>(obj)());
        }
        else if ( type == "VarMap" )
        {
            var.set(boost::python::extract<VarMap>(obj)());
        }
        else if ( type == "Var" )
        {
            var.set(boost::python::extract<Var>(obj)());
        }
        else if ( type == "Vec2" )
        {
            var.set(boost::python::extract<Vec2>(obj)());
        }
        else if ( type == "Rot" )
        {
            var.set(boost::python::extract<Rot>(obj)());
        }
        else if ( type == "Vec3" )
        {
            var.set(boost::python::extract<Vec3>(obj)());
        }
        else if ( type == "Quat" )
        {
            var.set(boost::python::extract<Quat>(obj)());
        }
        
        return var;
    }
    
    bool Var::empty()
    {
        return !m_data;
    }
    
    boost::python::object Var::pyGet()
    {
        return toPy();
    }
    
    void Var::pySet( boost::python::object obj )
    {
        *this = Var::fromPy(obj);
    }
    
    boost::python::object Var::defineClass()
    {
        return boost::python::class_<Var>("Var",boost::python::init<>())
            .def(boost::python::init<boost::python::object>())
            .def("empty",&Var::empty)
            .def("clear",&Var::clear)
            .def("get",&Var::pyGet)
            .def("set",&Var::pySet)
        ;
    }
}
