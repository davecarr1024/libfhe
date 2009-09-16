#include "PyFunc.h"
#include "VarMap.h"
#include <stdexcept>

namespace fhe
{
    
    IFuncWrapper* PyFuncUtil::bind( boost::python::object tret, boost::python::object targ, boost::python::object func )
    {
        if ( tret == boost::python::object() )
        {
            return bind2<void>(targ, func);
        }
        
        std::string type = boost::python::extract<std::string>(tret.attr("__name__"));
        
        if ( type == "bool" )
        {
            return bind2<bool>(targ, func);
        }
        else if ( type == "int" )
        {
            return bind2<int>(targ, func);
        }
        else if ( type == "float" )
        {
            return bind2<float>(targ, func);
        }
        else if ( type == "str" )
        {
            return bind2<std::string>(targ, func);
        }
        else if ( type == "VarMap" )
        {
            return bind2<VarMap>(targ, func);
        }
        else
        {
            std::string name = boost::python::extract<std::string>(func.attr("__name__"));
            throw std::runtime_error("unable to bind unknown ret type " + type + " for func " + name);
        }
    }
    
    template <class TRet>
    IFuncWrapper* PyFuncUtil::bind2( boost::python::object targ, boost::python::object func )
    {
        if ( targ == boost::python::object() )
        {
            return new PyFunc<TRet,void>(func);
        }
        
        std::string type = boost::python::extract<std::string>(targ.attr("__name__"));
        
        if ( type == "bool" )
        {
            return new PyFunc<TRet,bool>(func);
        }
        else if ( type == "int" )
        {
            return new PyFunc<TRet,int>(func);
        }
        else if ( type == "float" )
        {
            return new PyFunc<TRet,float>(func);
        }
        else if ( type == "str" )
        {
            return new PyFunc<TRet,std::string>(func);
        }
        else if ( type == "VarMap" )
        {
            return new PyFunc<TRet,VarMap>(func);
        }
        else
        {
            std::string name = boost::python::extract<std::string>(func.attr("__name__"));
            throw std::runtime_error("unable to bind unknown arg type " + type + " for func " + name);
        }
    }
    
}
