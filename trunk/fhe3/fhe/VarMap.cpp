#include "VarMap.h"

#include "PyConverter.h"

namespace fhe
{
    FHE_TO_CONVERTER(VarMap,const_cast<VarMap&>(obj).toPy());
    FHE_FROM_CONVERTER(VarMap,VarMap::fromPy(obj));
    
    VarMap::VarMap()
    {
    }
    
    VarMap::VarMap( boost::python::dict dict )
    {
        *this = VarMap::fromPy(dict);
    }

    void VarMap::removeVar( const std::string& name )
    {
        m_vars.erase(name);
    }
    
    void VarMap::clearVars()
    {
        m_vars.clear();
    }

    boost::python::object VarMap::toPy()
    {
        boost::python::dict dict;
        
        for ( std::map<std::string,Var>::iterator i = m_vars.begin(); i != m_vars.end(); ++i )
        {
            dict[i->first] = i->second.toPy();
        }
        
        return dict;
    }
    
    VarMap VarMap::fromPy( boost::python::object obj )
    {
        VarMap varMap;

        boost::python::object items = obj.attr("items")();
        
        for ( int i = 0; i < boost::python::len(items); ++i )
        {
            varMap.pySetVar(boost::python::extract<std::string>(items[i][0]),items[i][1]);
        }
        
        return varMap;
    }
    
    bool VarMap::pyHasVar( const std::string& name )
    {
        return m_vars.find(name) != m_vars.end();
    }
    
    boost::python::object VarMap::pyGetVar( const std::string& name )
    {
        return pyGetVarDef(name,boost::python::object());
    }
    
    boost::python::object VarMap::pyGetVarDef( const std::string& name, boost::python::object def )
    {
        Var var = onGetVar(name);
        if (!var.empty())
        {
            m_vars[name] = var;
        }
        
        if ( pyHasVar( name ) )
        {
            return m_vars[name].toPy();
        }
        else
        {
            return def;
        }
    }
    
    void VarMap::pySetVar( const std::string& name, boost::python::object val )
    {
        m_vars[name] = Var::fromPy(val);
        onSetVar(name,m_vars[name]);
    }
    
    boost::python::object VarMap::defineClass()
    {
        return boost::python::class_<VarMap>("VarMap",boost::python::init<>())
            .def(boost::python::init<boost::python::dict>())
            .def("removeVar",&VarMap::removeVar)
            .def("clearVars",&VarMap::clearVars)
            .def("hasVar",&VarMap::pyHasVar)
            .def("getVar",&VarMap::pyGetVar)
            .def("getVar",&VarMap::pyGetVarDef)
            .def("setVar",&VarMap::pySetVar)
        ;
    }
    
    Var VarMap::onGetVar( const std::string& name )
    {
        return Var();
    }
    
    void VarMap::onSetVar( const std::string& name, const Var& val )
    {
    }
}
