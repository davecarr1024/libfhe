#include "Var.h"
#include <cassert>

#include "math/Vec2.h"
#include "math/Rot.h"
#include "math/Mat3.h"
#include "math/Vec3.h"
#include "math/Quat.h"
#include "math/Mat4.h"
#include "VarMap.h"

namespace fhe
{
    
    Var::Var() :
        m_type(NONE)
    {
    }
    
    Var::~Var()
    {
        setType(NONE);
    }
    
    Var::Var( const Var& var ) :
        m_type( NONE )
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
            case VEC2:
                *m_data.v2 = *var.m_data.v2;
                break;
            case ROT:
                *m_data.r = *var.m_data.r;
                break;
            case MAT3:
                *m_data.m3 = *var.m_data.m3;
                break;
            case VEC3:
                *m_data.v3 = *var.m_data.v3;
                break;
            case QUAT:
                *m_data.q = *var.m_data.q;
                break;
            case MAT4:
                *m_data.m4 = *var.m_data.m4;
                break;
            case VARMAP:
                *m_data.vm = *var.m_data.vm;
                break;
        }
    }
   
    Var& Var::operator=( const Var& var )
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
            case VEC2:
                *m_data.v2 = *var.m_data.v2;
                break;
            case ROT:
                *m_data.r = *var.m_data.r;
                break;
            case MAT3:
                *m_data.m3 = *var.m_data.m3;
                break;
            case VEC3:
                *m_data.v3 = *var.m_data.v3;
                break;
            case QUAT:
                *m_data.q = *var.m_data.q;
                break;
            case MAT4:
                *m_data.m4 = *var.m_data.m4;
                break;
            case VARMAP:
                *m_data.vm = *var.m_data.vm;
                break;
        }
        return *this;
    }
    
    Var::Type Var::getType()
    {
        return m_type;
    }
    
    void Var::setType( Type type )
    {
        switch ( m_type )
        {
            case STRING:
                delete m_data.s;
                break;
            case VEC2:
                delete m_data.v2;
                break;
            case ROT:
                delete m_data.r;
                break;
            case MAT3:
                delete m_data.m3;
                break;
            case VEC3:
                delete m_data.v3;
                break;
            case QUAT:
                delete m_data.q;
                break;
            case MAT4:
                delete m_data.m4;
                break;
            case VARMAP:
                delete m_data.vm;
                break;
            default:
                break;
        }
        
        m_type = type;
        
        switch ( m_type )
        {
            case STRING:
                m_data.s = new std::string();
                break;
            case VEC2:
                m_data.v2 = new Vec2();
                break;
            case ROT:
                m_data.r = new Rot();
                break;
            case MAT3:
                m_data.m3 = new Mat3();
                break;
            case VEC3:
                m_data.v3 = new Vec3();
                break;
            case QUAT:
                m_data.q = new Quat();
                break;
            case MAT4:
                m_data.m4 = new Mat4();
                break;
            case VARMAP:
                m_data.vm = new VarMap();
                break;
            default:
                break;
        }
    }

    template<>
    bool Var::is<bool>()
    {
        return m_type == BOOL;
    }

    template<>
    bool Var::is<int>()
    {
        return m_type == INT;
    }

    template<>
    bool Var::is<float>()
    {
        return m_type == FLOAT;
    }

    template<>
    bool Var::is<std::string>()
    {
        return m_type == STRING;
    }

    template<>
    bool Var::is<Vec2>()
    {
        return m_type == VEC2;
    }

    template<>
    bool Var::is<Rot>()
    {
        return m_type == ROT;
    }

    template<>
    bool Var::is<Mat3>()
    {
        return m_type == MAT3;
    }

    template<>
    bool Var::is<Vec3>()
    {
        return m_type == VEC3;
    }

    template<>
    bool Var::is<Quat>()
    {
        return m_type == QUAT;
    }

    template<>
    bool Var::is<Mat4>()
    {
        return m_type == MAT4;
    }
    
    template<>
    bool Var::is<VarMap>()
    {
        return m_type == VARMAP;
    }

    template<>
    void Var::set<bool>( const bool& val )
    {
        setType(BOOL);
        m_data.b = val;
    }
    
    template<>
    void Var::set<int>( const int& val )
    {
        setType(INT);
        m_data.i = val;
    }
    
    template<>
    void Var::set<float>( const float& val )
    {
        setType(FLOAT);
        m_data.f = val;
    }
    
    template<>
    void Var::set<std::string>( const std::string& val )
    {
        setType(STRING);
        *m_data.s = val;
    }
    
    template<>
    void Var::set<Vec2>( const Vec2& val )
    {
        setType(VEC2);
        *m_data.v2 = val;
    }
    
    template<>
    void Var::set<Rot>( const Rot& val )
    {
        setType(ROT);
        *m_data.r = val;
    }
    
    template<>
    void Var::set<Mat3>( const Mat3& val )
    {
        setType(MAT3);
        *m_data.m3 = val;
    }
    
    template<>
    void Var::set<Vec3>( const Vec3& val )
    {
        setType(VEC3);
        *m_data.v3 = val;
    }
    
    template<>
    void Var::set<Quat>( const Quat& val )
    {
        setType(QUAT);
        *m_data.q = val;
    }
    
    template<>
    void Var::set<Mat4>( const Mat4& val )
    {
        setType(MAT4);
        *m_data.m4 = val;
    }
    
    template<>
    void Var::set<VarMap>( const VarMap& val )
    {
        setType(VARMAP);
        *m_data.vm = val;
    }
    
    template<>
    bool Var::get<bool>()
    {
        assert(is<bool>());
        return m_data.b;
    }

    template<>
    int Var::get<int>()
    {
        assert(is<int>());
        return m_data.i;
    }

    template<>
    float Var::get<float>()
    {
        assert(is<float>());
        return m_data.f;
    }

    template<>
    std::string Var::get<std::string>()
    {
        assert(is<std::string>());
        return *m_data.s;
    }

    template<>
    Vec2 Var::get<Vec2>()
    {
        assert(is<Vec2>());
        return *m_data.v2;
    }

    template<>
    Rot Var::get<Rot>()
    {
        assert(is<Rot>());
        return *m_data.r;
    }

    template<>
    Mat3 Var::get<Mat3>()
    {
        assert(is<Mat3>());
        return *m_data.m3;
    }

    template<>
    Vec3 Var::get<Vec3>()
    {
        assert(is<Vec3>());
        return *m_data.v3;
    }

    template<>
    Quat Var::get<Quat>()
    {
        assert(is<Quat>());
        return *m_data.q;
    }

    template<>
    Mat4 Var::get<Mat4>()
    {
        assert(is<Mat4>());
        return *m_data.m4;
    }
    
    template<>
    VarMap Var::get<VarMap>()
    {
        assert(is<VarMap>());
        return *m_data.vm;
    }

    boost::python::object Var::toPy()
    {
        switch ( m_type )
        {
            case NONE:
                return boost::python::object();
            case BOOL:
                return boost::python::object(m_data.b);
            case INT:
                return boost::python::object(m_data.i);
            case FLOAT:
                return boost::python::object(m_data.f);
            case STRING:
                return boost::python::object(*m_data.s);
            case VEC2:
                return boost::python::object(*m_data.v2);
            case ROT:
                return boost::python::object(*m_data.r);
            case MAT3:
                return boost::python::object(*m_data.m3);
            case VEC3:
                return boost::python::object(*m_data.v3);
            case QUAT:
                return boost::python::object(*m_data.q);
            case MAT4:
                return boost::python::object(*m_data.m4);
            case VARMAP:
                return m_data.vm->toPy();
        }
    }
    
    Var Var::fromPy( boost::python::object obj )
    {
        Var var;
        std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
        
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
        else if ( type == "Vec2" )
        {
            var.set<Vec2>(boost::python::extract<Vec2>(obj));
        }
        else if ( type == "Rot" )
        {
            var.set<Rot>(boost::python::extract<Rot>(obj));
        }
        else if ( type == "Mat3" )
        {
            var.set<Mat3>(boost::python::extract<Mat3>(obj));
        }
        else if ( type == "Vec3" )
        {
            var.set<Vec3>(boost::python::extract<Vec3>(obj));
        }
        else if ( type == "Quat" )
        {
            var.set<Quat>(boost::python::extract<Quat>(obj));
        }
        else if ( type == "Mat4" )
        {
            var.set<Mat4>(boost::python::extract<Mat4>(obj));
        }
        else if ( type == "VarMap" )
        {
            var.set<VarMap>(VarMap::fromPy(obj));
        }
        
        return var;
    }
}
