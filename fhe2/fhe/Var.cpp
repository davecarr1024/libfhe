#include "Var.h"
#include "VarMap.h"

#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Vec3.h"
#include "math/Quat.h"

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
            case VEC2:
                *m_data.v2 = *var.m_data.v2;
                break;
            case ROT:
                *m_data.r = *var.m_data.r;
                break;
            case VEC3:
                *m_data.v3 = *var.m_data.v3;
                break;
            case QUAT:
                *m_data.q = *var.m_data.q;
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
            case VEC2:
                *m_data.v2 = *var.m_data.v2;
                break;
            case ROT:
                *m_data.r = *var.m_data.r;
                break;
            case VEC3:
                *m_data.v3 = *var.m_data.v3;
                break;
            case QUAT:
                *m_data.q = *var.m_data.q;
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
                case VEC2:
                    delete m_data.v2;
                    break;
                case ROT:
                    delete m_data.r;
                    break;
                case VEC3:
                    delete m_data.v3;
                    break;
                case QUAT:
                    delete m_data.q;
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
                case VEC2:
                    m_data.v2 = new Vec2();
                    break;
                case ROT:
                    m_data.r = new Rot();
                    break;
                case VEC3:
                    m_data.v3 = new Vec3();
                    break;
                case QUAT:
                    m_data.q = new Quat();
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
    bool Var::is<Vec2>()
    {
        return m_type == VEC2;
    }
    
    template <>
    bool Var::is<Rot>()
    {
        return m_type == ROT;
    }
    
    template <>
    bool Var::is<Vec3>()
    {
        return m_type == VEC3;
    }
    
    template <>
    bool Var::is<Quat>()
    {
        return m_type == QUAT;
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
    
    template <>
    Vec2 Var::get<Vec2>()
    {
        assert(is<Vec2>());
        return *m_data.v2;
    }
    
    template <>
    Rot Var::get<Rot>()
    {
        assert(is<Rot>());
        return *m_data.r;
    }
    
    template <>
    Vec3 Var::get<Vec3>()
    {
        assert(is<Vec3>());
        return *m_data.v3;
    }
    
    template <>
    Quat Var::get<Quat>()
    {
        assert(is<Quat>());
        return *m_data.q;
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
    
    template <>
    void Var::set<Vec2>( const Vec2& val )
    {
        setType(VEC2);
        *m_data.v2 = val;
    }
    
    template <>
    void Var::set<Rot>( const Rot& val )
    {
        setType(ROT);
        *m_data.r = val;
    }
    
    template <>
    void Var::set<Vec3>( const Vec3& val )
    {
        setType(VEC3);
        *m_data.v3 = val;
    }
    
    template <>
    void Var::set<Quat>( const Quat& val )
    {
        setType(QUAT);
        *m_data.q = val;
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
        else if ( type == "Vec2" )
        {
            var.set<Vec2>(boost::python::extract<Vec2>(obj));
        }
        else if ( type == "Rot" )
        {
            var.set<Rot>(boost::python::extract<Rot>(obj));
        }
        else if ( type == "Vec3" )
        {
            var.set<Vec3>(boost::python::extract<Vec3>(obj));
        }
        else if ( type == "Quat" )
        {
            var.set<Quat>(boost::python::extract<Quat>(obj));
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
