#include "PyEnv.h"
#include "PyEntity.h"

#include <fhe/math/Vec2.h>
#include <fhe/math/Vec3.h>
#include <fhe/math/Rot.h>
#include <fhe/math/Quat.h>
#include <fhe/VarList.h>
#include <fhe/VarMap.h>
#include <fhe/FileSystem.h>

namespace fhe
{
    namespace Python
    {
        
        PyEnv::PyEnv()
        {
            Py_Initialize();
            
            m_mainModule = boost::python::import("__main__");
            m_mainNamespace = boost::python::dict(m_mainModule.attr("__dict__"));
            m_builtins = m_mainNamespace["__builtins__"].attr("__dict__");
            
            m_mainNamespace["Vec2"] = Vec2::defineClass();
            m_mainNamespace["Vec3"] = Vec3::defineClass();
            m_mainNamespace["Rot"] = Rot::defineClass();
            m_mainNamespace["Quat"] = Quat::defineClass();
            
            PyEntity::defineClass();
        }
        
        PyEnv& PyEnv::instance()
        {
            static PyEnv pe;
            return pe;
        }
        
        boost::python::dict PyEnv::defaultNamespace()
        {
            boost::python::dict ns;
            ns.update(m_mainNamespace);
            return ns;
        }
        
        void PyEnv::runFile( const std::string& filename, boost::python::dict ns )
        {
            try
            {
                boost::python::exec_file(FileSystem::instance().getFile(filename).c_str(),ns,ns);
            }
            catch ( boost::python::error_already_set )
            {
                PyErr_Print();
                PyErr_Clear();
                throw std::runtime_error( "error running python file " + filename );
            }
        }
        
        void PyEnv::exec( const std::string& script, boost::python::dict ns )
        {
            try
            {
                boost::python::exec(script.c_str(),ns,ns);
            }
            catch ( boost::python::error_already_set )
            {
                PyErr_Print();
                PyErr_Clear();
                throw std::runtime_error( "error execing python script " + script );
            }
        }
        
        boost::python::object PyEnv::eval( const std::string& script, boost::python::dict ns )
        {
            try
            {
                return boost::python::eval(script.c_str(),ns,ns);
            }
            catch ( boost::python::error_already_set )
            {
                PyErr_Print();
                PyErr_Clear();
                throw std::runtime_error( "error evaling python script " + script );
            }
        }
        
        std::string PyEnv::getType( boost::python::object obj )                                    
        {                                                                                        
            return boost::python::extract<std::string>(m_builtins["type"](obj).attr("__name__"));
        }                                                                                        
                                                                                                 
        Var PyEnv::convertToVar( boost::python::object obj )                                       
        {                                                                                        
            if ( obj == boost::python::object() )                                                
            {                                                                                    
                return Var();                                                                    
            }                                                                                    
            else                                                                                 
            {                                                                                    
                std::string type = getType(obj);                                                 
                                                                                                 
                if ( type == "bool" )                                                            
                {                                                                                
                    return Var::build<bool>(boost::python::extract<bool>(obj)());                
                }                                                                                
                else if ( type == "int" )                                                        
                {                                                                                
                    return Var::build<int>(boost::python::extract<int>(obj)());                  
                }                                                                                
                else if ( type == "float" )                                                      
                {                                                                                
                    return Var::build<float>(boost::python::extract<float>(obj)());              
                }                                                                                
                else if ( type == "str" )                                                        
                {                                                                                
                    return Var::build<std::string>(boost::python::extract<std::string>(obj)());  
                }                                                                                
                else if ( type == "Vec2" )                                                       
                {                                                                                
                    return Var::build<Vec2>(boost::python::extract<Vec2>(obj)());                
                }                                                                                
                else if ( type == "Vec3" )                                                       
                {                                                                                
                    return Var::build<Vec3>(boost::python::extract<Vec3>(obj)());                
                }                                                                                
                else if ( type == "Rot" )                                                        
                {                                                                                
                    return Var::build<Rot>(boost::python::extract<Rot>(obj)());                  
                }                                                                                
                else if ( type == "Quat" )                                                       
                {                                                                                
                    return Var::build<Quat>(boost::python::extract<Quat>(obj)());                
                }                                                                                
                else if ( type == "dict" )                                                       
                {                                                                                
                    VarMap vm;                                                                   
                    boost::python::object items = obj.attr("items")();                           
                    for ( int i = 0; i < boost::python::len(items); ++i )                        
                    {                                                                            
                        vm._setVar(boost::python::extract<std::string>(items[i][0]),convertToVar(items[i][1]));                                                                                           
                    }                                                                                 
                    return Var::build<VarMap>(vm);                                                    
                }
                else if ( type == "list" )
                {
                    VarList vl;
                    for ( int i = 0; i < boost::python::len(obj); ++i )
                    {
                        vl.appendRaw( convertToVar(obj[i]) );
                    }
                    return Var::build<VarList>(vl);
                }
                else                                                                                  
                {                                                                                     
                    throw std::runtime_error("unable to convert unknown python type " + type + " to var");                                                                                                  
                }                                                                                     
            }                                                                                         
        }                                                                                             
                                                                                                      
        boost::python::object PyEnv::convertFromVar( const Var& val )                                   
        {                                                                                             
            if ( val.empty() )                                                                        
            {                                                                                         
                return boost::python::object();                                                       
            }                                                                                         
            else                                                                                      
            {                                                                                         
                if ( val.is<bool>() )                                                                 
                {                                                                                     
                    return boost::python::object(val.get<bool>());                                    
                }                                                                                     
                else if ( val.is<int>() )                                                             
                {                                                                                     
                    return boost::python::object(val.get<int>());                                     
                }                                                                                     
                else if ( val.is<float>() )                                                           
                {                                                                                     
                    return boost::python::object(val.get<float>());                                   
                }                                                                                     
                else if ( val.is<std::string>() )                                                     
                {                                                                                     
                    return boost::python::object(val.get<std::string>());                             
                }                                                                                     
                else if ( val.is<Vec2>() )                                                            
                {                                                                                     
                    return boost::python::object(val.get<Vec2>());                                    
                }                                                                                     
                else if ( val.is<Vec3>() )                                                            
                {                                                                                     
                    return boost::python::object(val.get<Vec3>());                                    
                }                                                                                     
                else if ( val.is<Rot>() )                                                             
                {                                                                                     
                    return boost::python::object(val.get<Rot>());                                     
                }                                                                                     
                else if ( val.is<Quat>() )
                {
                    return boost::python::object(val.get<Quat>());
                }
                else if ( val.is<VarMap>() )
                {
                    boost::python::dict d;
                    VarMap vm = val.get<VarMap>();
                    std::vector<std::string> names = vm.getVarNames();
                    for ( std::vector<std::string>::iterator i = names.begin(); i != names.end(); ++i)
                    {
                        d[*i] = convertFromVar(vm._getVar(*i));
                    }
                    return d;
                }
                else if ( val.is<VarList>() )
                {
                    boost::python::list l;
                    VarList vl = val.get<VarList>();
                    for ( int i = 0; i < vl.length(); ++i )
                    {
                        l.append(convertFromVar(vl.getRawVar(i)));
                    }
                    return l;
                }
                else
                {
                    throw std::runtime_error("unable to convert unknown c type to python");
                }
            }
        }
     }
}
