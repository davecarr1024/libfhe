#include "VarMap.h"
#include "math/Vec2.h"
#include "math/Vec3.h"
#include "math/Rot.h"
#include "math/Quat.h"

namespace fhe
{
    
    VarMap::VarMap()
    {
    }
    
    VarMap::VarMap( const VarMap& val )
    {
        clone(val);
    }
    
    VarMap::VarMap( boost::python::object obj )
    {
        *this = VarMap::fromPy( obj );
    }
    
    VarMap& VarMap::operator=( const VarMap& val )
    {
        clone(val);
        return *this;
    }
    
    VarMap::~VarMap()
    {
        clearVars();
    }
    
    void VarMap::clone( const VarMap& _val )
    {
        VarMap& val = const_cast<VarMap&>(_val);
        
        clearVars();
        
        std::vector<std::string> names = val.getVarNames();
        
        for ( std::vector<std::string>::iterator i = names.begin(); i != names.end(); ++i )
        {
            if ( val.hasVar<bool>(*i) )
            {
                setVar<bool>(*i,val.getVar<bool>(*i));
            }
            else if ( val.hasVar<int>(*i) )
            {
                setVar<int>(*i,val.getVar<int>(*i));
            }
            else if ( val.hasVar<float>(*i) )
            {
                setVar<float>(*i,val.getVar<float>(*i));
            }
            else if ( val.hasVar<std::string>(*i) )
            {
                setVar<std::string>(*i,val.getVar<std::string>(*i));
            }
            else if ( val.hasVar<VarMap>(*i) )
            {
                setVar<VarMap>(*i,val.getVar<VarMap>(*i));
            }
            else if ( val.hasVar<Vec2>(*i) )
            {
                setVar<Vec2>(*i,val.getVar<Vec2>(*i));
            }
            else if ( val.hasVar<Vec3>(*i) )
            {
                setVar<Vec3>(*i,val.getVar<Vec3>(*i));
            }
            else if ( val.hasVar<Rot>(*i) )
            {
                setVar<Rot>(*i,val.getVar<Rot>(*i));
            }
            else if ( val.hasVar<Quat>(*i) )
            {
                setVar<Quat>(*i,val.getVar<Quat>(*i));
            }
            else
            {
                throw std::runtime_error("ERROR: can't copy " + *i + " to a new VarMap!");
            }
        }
    }
    
    boost::python::object VarMap::defineClass()
    {
        return boost::python::class_<VarMap>("VarMap", boost::python::init<>() )
            .def(boost::python::init<boost::python::object>())
            .def("pop",&VarMap::removeVar)
            .def("clear",&VarMap::clearVars)
            .def("keys",&VarMap::pyGetVarNames)
            .def("__contains__",&VarMap::pyHasVar)
            .def("__getitem__",&VarMap::pyGetVar)
            .def("__setitem__",&VarMap::pySetVar)
            .def("get",&VarMap::pyGetVarDef)
        ;
    }
    
    void VarMap::removeVar( const std::string& name )
    {
        if ( m_vars.find( name ) != m_vars.end() )
        {
            delete m_vars[name];
            m_vars.erase( name );
        }
    }

    void VarMap::clearVars()
    {
        for (std::map<std::string, IVarWrapper*>::iterator i = m_vars.begin(); i != m_vars.end(); ++i)
        {
            delete i->second;
        }
        m_vars.clear();
    }
    
    std::vector<std::string> VarMap::getVarNames()
    {
        std::vector<std::string> names;
        for ( std::map<std::string, IVarWrapper*>::iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            names.push_back( i->first );
        }
        return names;
    }

    boost::python::object VarMap::toPy()
    {
        boost::python::dict d;
        std::string name;
        for ( std::map<std::string, IVarWrapper*>::iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            name = i->first;
            if ( hasVar<bool>(name) )
            {
                d[name] = boost::python::object( getVar<bool>(name) );
            }
            else if ( hasVar<int>(name) )
            {
                d[name] = boost::python::object( getVar<int>(name) );
            }
            else if ( hasVar<float>(name) )
            {
                d[name] = boost::python::object( getVar<float>(name) );
            }
            else if ( hasVar<std::string>(name) )
            {
                d[name] = boost::python::object( getVar<std::string>(name) );
            }
            else if ( hasVar<VarMap>(name) )
            {
                d[name] = getVar<VarMap>(name).toPy();
            }
            else if ( hasVar<Vec2>(name) )
            {
                d[name] = boost::python::object( getVar<Vec2>(name) );
            }
            else if ( hasVar<Vec3>(name) )
            {
                d[name] = boost::python::object( getVar<Vec3>(name) );
            }
            else if ( hasVar<Rot>(name) )
            {
                d[name] = boost::python::object( getVar<Rot>(name) );
            }
            else if ( hasVar<Quat>(name) )
            {
                d[name] = boost::python::object( getVar<Quat>(name) );
            }
        }
        return d;
    }
    
    VarMap VarMap::fromPy( boost::python::object obj )
    {
        VarMap val;
        std::string dictType = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
        if ( dictType == "dict" )
        {
            boost::python::object items = obj.attr("items")();
            int l = boost::python::extract<int>(items.attr("__len__")());
            bool b;
            int i;
            float f;
            std::string s;
            std::string name;
            std::string type;
            for ( int i = 0; i < l; ++i )
            {
                name = boost::python::extract<std::string>(items[i][0]);
                type = boost::python::extract<std::string>(items[i][1].attr("__class__").attr("__name__"));
                if ( type == "bool" )
                {
                    val.setVar<bool>(name,boost::python::extract<bool>(items[i][1]));
                }
                else if ( type == "int" )
                {
                    val.setVar<int>(name,boost::python::extract<int>(items[i][1]));
                }
                else if ( type == "float" )
                {
                    val.setVar<float>(name,boost::python::extract<float>(items[i][1]));
                }
                else if ( type == "str" )
                {
                    val.setVar<std::string>(name,boost::python::extract<std::string>(items[i][1]));
                }
                else if ( type == "VarMap" )
                {
                    val.setVar<VarMap>(name,boost::python::extract<VarMap>(items[i][1]));
                }
                else if ( type == "Vec2" )
                {
                    val.setVar<Vec2>(name,boost::python::extract<Vec2>(items[i][1]));
                }
                else if ( type == "Vec3" )
                {
                    val.setVar<Vec3>(name,boost::python::extract<Vec3>(items[i][1]));
                }
                else if ( type == "Rot" )
                {
                    val.setVar<Rot>(name,boost::python::extract<Rot>(items[i][1]));
                }
                else if ( type == "Quat" )
                {
                    val.setVar<Quat>(name,boost::python::extract<Quat>(items[i][1]));
                }
            }
        }
        else
        {
            throw std::runtime_error( "ERROR: can't build VarMap from non-dict type " + dictType );
        }
        return val;
    }
    
    std::string VarMap::getType( boost::python::object obj )
    {
        return boost::python::extract<std::string>( obj.attr("__class__").attr("__name__") );
    }
    
    bool VarMap::pyHasVar( const std::string& name )
    {
        return hasVar<bool>(name) ||
               hasVar<int>(name) ||
               hasVar<float>(name) ||
               hasVar<std::string>(name) ||
               hasVar<VarMap>(name) ||
               hasVar<Vec2>(name) ||
               hasVar<Vec3>(name) ||
               hasVar<Rot>(name) ||
               hasVar<Quat>(name)
               ;
    }
    
    void VarMap::pySetVar( const std::string& name, boost::python::object val )
    {
        std::string type = getType(val);
        
        if ( type == "bool" )
        {
            setVar<bool>(name,boost::python::extract<bool>(val));
        }
        else if ( type == "int" )
        {
            setVar<int>(name,boost::python::extract<int>(val));
        }
        else if ( type == "float" )
        {
            setVar<float>(name,boost::python::extract<float>(val));
        }
        else if ( type == "str" )
        {
            setVar<std::string>(name,boost::python::extract<std::string>(val));
        }
        else if ( type == "VarMap" )
        {
            setVar<VarMap>(name,boost::python::extract<VarMap>(val));
        }
        else if ( type == "Vec2" )
        {
            setVar<Vec2>(name,boost::python::extract<Vec2>(val));
        }
        else if ( type == "Vec3" )
        {
            setVar<Vec3>(name,boost::python::extract<Vec3>(val));
        }
        else if ( type == "Rot" )
        {
            setVar<Rot>(name,boost::python::extract<Rot>(val));
        }
        else if ( type == "Quat" )
        {
            setVar<Quat>(name,boost::python::extract<Quat>(val));
        }
        else
        {
            throw std::runtime_error( "unable to set var " + name + " to unknown python type " + type );
        }
    }
    
    boost::python::object VarMap::pyGetVar( const std::string& name )
    {
        return pyGetVarDef( name, boost::python::object() );
    }
    
    boost::python::object VarMap::pyGetVarDef( const std::string& name, boost::python::object def )
    {
        if ( hasVar<bool>(name) )
        {
            return boost::python::object( getVar<bool>(name) );
        }
        else if ( hasVar<int>(name) )
        {
            return boost::python::object( getVar<int>(name) );
        }
        else if ( hasVar<float>(name) )
        {
            return boost::python::object( getVar<float>(name) );
        }
        else if ( hasVar<std::string>(name) )
        {
            return boost::python::object( getVar<std::string>(name) );
        }
        else if ( hasVar<VarMap>(name) )
        {
            return boost::python::object( getVar<VarMap>(name) );
        }
        else if ( hasVar<Vec2>(name) )
        {
            return boost::python::object( getVar<Vec2>(name) );
        }
        else if ( hasVar<Vec3>(name) )
        {
            return boost::python::object( getVar<Vec3>(name) );
        }
        else if ( hasVar<Rot>(name) )
        {
            return boost::python::object( getVar<Rot>(name) );
        }
        else if ( hasVar<Quat>(name) )
        {
            return boost::python::object( getVar<Quat>(name) );
        }
        else
        {
            return def;
        }
    }
    
    boost::python::object VarMap::pyGetVarNames()
    {
        std::vector<std::string> names = getVarNames();
        boost::python::list pyNames;
        for ( std::vector<std::string>::iterator i = names.begin(); i != names.end(); ++i )
        {
            pyNames.append( *i );
        }
        return pyNames;
    }

    template <class T>
    bool VarMap::hasVar( const std::string& name )
    {
        return m_vars.find(name) != m_vars.end() && m_vars[name]->cast<T>();
    }

    template <class T>
    bool VarMap::canSetVar( const std::string& name )
    {
        return hasVar<T>(name) && m_vars[name]->cast<T>()->canSet();
    }

    template <class T>
    bool VarMap::canGetVar( const std::string& name )
    {
        return hasVar<T>(name) && m_vars[name]->cast<T>()->canGet();
    }

    template <class T>
    void VarMap::setVar( const std::string& name, const T& val )
    {
        if ( hasVar<T>( name ) )
        {
            IVar<T>* var = m_vars[name]->cast<T>();
            assert( var->canSet() );
            var->set( val );
        }
        else
        {
            connectVar( name, new Var<T>( val ) );
        }
    }

    template <class T>
    void VarMap::connectVar( const std::string& name, IVar<T>* val )
    {
        assert(val);
        removeVar( name );
        m_vars[name] = val;
    }

    template <class T>
    T VarMap::getVar( const std::string& name )
    {
        assert( canGetVar<T>(name) );
        return m_vars[name]->cast<T>()->get();
    }

    template <class T>
    T VarMap::getVar( const std::string& name, const T& def )
    {
        if ( canGetVar<T>(name) )
        {
            return m_vars[name]->cast<T>()->get();
        }
        else
        {
            return def;
        }
    }
}
