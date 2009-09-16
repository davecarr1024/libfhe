#include "Var.h"
#include "VarMap.h"
#include <stdexcept>

namespace fhe
{
    
    IVarWrapper* IVarWrapper::newPy( boost::python::object obj )
    {
        std::string type = boost::python::extract<std::string>(obj.attr("__class__").attr("__name__"));
        if ( type == "bool" )
        {
            return new Var<bool>(boost::python::extract<bool>(obj));
        }
        else if ( type == "int" )
        {
            return new Var<int>(boost::python::extract<int>(obj));
        }
        else if ( type == "float" )
        {
            return new Var<float>(boost::python::extract<float>(obj));
        }
        else if ( type == "str" )
        {
            return new Var<std::string>(boost::python::extract<std::string>(obj));
        }
        else if ( type == "VarMap" )
        {
            return new Var<VarMap>(boost::python::extract<VarMap>(obj));
        }
        else
        {
            throw new std::runtime_error("Can't convert python type " + type);
        }
    }
    
}
