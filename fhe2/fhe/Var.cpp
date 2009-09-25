#include "Var.h"
#include "VarMap.h"
#include <cassert>

namespace fhe
{
    
    Var::Var() :
        m_type(NONE)
    {
    }
    
    Var::Var( const Var& var ) :
        m_type(NONE)
    {
        setType( var.m_type );
        
        switch ( m_type )
        {
            case BOOL:
                m_data.b = var.m_data.b;
                break;
            case INT:
                m_data.i = var.m_data.i;
                break;
            case FLOAT:
                m_data.f = var.m_data.f;
                break;
            case STRING:
                *m_data.s = *var.m_data.s;
                break;
            case VARMAP:
                *m_data.vm = *var.m_data.vm;
                break;
        }
    }
    
    Var& Var::operator=( const Var& var )
    {
        m_type = NONE;
        setType( var.m_type );
        
        switch ( m_type )
        {
            case BOOL:
                m_data.b = var.m_data.b;
                break;
            case INT:
                m_data.i = var.m_data.i;
                break;
            case FLOAT:
                m_data.f = var.m_data.f;
                break;
            case STRING:
                *m_data.s = *var.m_data.s;
                break;
            case VARMAP:
                *m_data.vm = *var.m_data.vm;
                break;
        }
        
        return *this;
    }
    
    Var::Var( boost::python::object obj ) :
        m_type(NONE)
    {
        *this = Var::fromPy(obj);
    }
    
    Var::~Var()
    {
        setType(NONE);
    }
    
    Var::Type Var::getType()
    {
        return m_type;
    }
    
    void Var::setType( Type type )
    {
        if ( type != m_type )
        {
            switch ( m_type )
            {
                case STRING:
                    delete m_data.s;
                    break;
                case VARMAP:
                    delete m_data.vm;
                    break;
            }
            m_type = type;
            switch ( m_type )
            {
                case STRING:
                    m_data.s = new std::string();
                    break;
                case VARMAP:
                    m_data.vm = new VarMap();
                    break;
            }
        }
    }
    
    template <>
    bool Var::is<bool>()
    {
        return m_type == BOOL;
    }
    
    template <>
    bool Var::is<int>()
    {
        return m_type == INT;
    }
    
    template <>
    bool Var::is<float>()
    {
        return m_type == FLOAT;
    }
    
    template <>
    bool Var::is<std::string>()
    {
        return m_type == STRING;
    }
    
    template <>
    bool Var::is<VarMap>()
    {
        return m_type == VARMAP;
    }
    
    template <>
    bool Var::get<bool>()
    {
        assert(is<bool>());
        return m_data.b;
    }
    
    template <>
    int Var::get<int>()
    {
        assert(is<int>());
        return m_data.i;
    }
    
    template <>
    float Var::get<float>()
    {
        assert(is<float>());
        return m_data.f;
    }
    
    template <>
    std::string Var::get<std::string>()
    {
        assert(is<std::string>());
        return *m_data.s;
    }
    
    template <>
    VarMap Var::get<VarMap>()
    {
        assert(is<VarMap>());
        return *m_data.vm;
    }
    
    template <class T>
    T Var::get( const T& def )
    {
        if ( is<T>() )
        {
            return get<T>();
        }
        else
        {
            return def;
        }
    }
    
    template <>
    void Var::set<bool>( const bool& val )
    {
        setType(BOOL);
        m_data.b = val;
    }

    template <>
    void Var::set<int>( const int& val )
    {
        setType(INT);
        m_data.i = val;
    }

    template <>
    void Var::set<float>( const float& val )
    {
        setType(FLOAT);
        m_data.f = val;
    }

    template <>
    void Var::set<std::string>( const std::string& val )
    {
        setType(STRING);
        *m_data.s = val;
    }
    
    template <>
    void Var::set<VarMap>( const VarMap& val )
    {
        setType(VARMAP);
        *m_data.vm = val;
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
            return get<VarMap>().toPy();
        }
        else
        {
            return boost::python::object();
        }
    }
    
    Var Var::fromPy( boost::python::object obj )
    {
        std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
        
        Var var;
        
        if ( type == "bool" )
        {
            var.set<bool>(boost::python::extract<bool>(obj));
        }
        else if ( type == "int" )
        {
            var.set<int>(boost::python::extract<int>(obj));
        }
        else if ( type == "float" )
        {
            var.set<float>(boost::python::extract<float>(obj));
        }
        else if ( type == "str" )
        {
            var.set<std::string>(boost::python::extract<std::string>(obj));
        }
        else if ( type == "VarMap" )
        {
            var.set<VarMap>(VarMap::fromPy(obj));
        }
        
        return var;
    }
    
    boost::python::object Var::pyGet()
    {
        return toPy();
    }
    
    void Var::pySet( boost::python::object val )
    {
        *this = Var::fromPy(val);
    }
    
    boost::python::object Var::defineClass()
    {
        return boost::python::class_<Var>("Var",boost::python::init<>())
            .def(boost::python::init<boost::python::object>())
            .def("get",&Var::pyGet)
            .def("set",&Var::pySet)
        ;
    }
}
