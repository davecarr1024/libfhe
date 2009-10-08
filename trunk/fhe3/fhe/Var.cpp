#include "Var.h"
#include "VarMap.h"
#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Vec3.h"
#include "math/Quat.h"

namespace fhe
{
    FHE_TO_CONVERTER(Var,const_cast<Var&>(obj).toPy());
    FHE_FROM_CONVERTER(Var,Var::fromPy(obj));
    
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
        return m_data ? m_data->toPy() : boost::python::object();
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
