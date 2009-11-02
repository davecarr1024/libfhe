#include "Var.h"
#include "math/Vec3.h"
#include "math/Quat.h"

namespace gge
{
    
    GGE_TO_PYTHON_CONVERTER(Var,obj.toPy());
    GGE_FROM_PYTHON_CONVERTER(Var,Var::fromPy(obj));

    Var::Var() :
        m_data(0)
    {
    }
    
    Var::Var( const Var& var ) :
        m_data(var.m_data ? var.m_data->clone() : 0)
    {
    }
    
    Var& Var::operator=( const Var& var )
    {
        clear();
        m_data = var.m_data ? var.m_data->clone() : 0;
    }
    
    Var::~Var()
    {
        clear();
    }
    
    bool Var::empty()
    {
        return !m_data;
    }
    
    void Var::clear()
    {
        if ( m_data )
        {
            delete m_data;
            m_data = 0;
        }
    }
    
    boost::python::object Var::toPy() const
    {
        return m_data ? m_data->toPy() : boost::python::object();
    }
    
    Var Var::fromPy( boost::python::object obj )
    {
        Var var;
        if ( obj != boost::python::object() )
        {
            std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
            if ( type == "bool" )
            {
                var.set<bool>(boost::python::extract<bool>(obj)());
            }
            else if ( type == "int" )
            {
                var.set<int>(boost::python::extract<int>(obj)());
            }
            else if ( type == "float" )
            {
                var.set<float>(boost::python::extract<float>(obj)());
            }
            else if ( type == "str" )
            {
                var.set<std::string>(boost::python::extract<std::string>(obj)());
            }
            else if ( type == "Vec3" )
            {
                var.set<Vec3>(boost::python::extract<Vec3>(obj)());
            }
            else if ( type == "Quat" )
            {
                var.set<Quat>(boost::python::extract<Quat>(obj)());
            }
            else
            {
                printf("warning: cannot convert unknown python type %s to var\n",type.c_str());
            }
        }
        return var;
    }
    
    std::string Var::toString() const
    {
        return m_data ? m_data->toString() : "None";
    }
}
