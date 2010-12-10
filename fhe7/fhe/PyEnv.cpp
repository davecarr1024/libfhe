#include <fhe/PyEnv.h>
#include <fhe/PyNode.h>
#include <fhe/FileSystem.h>
#include <fhe/Vec.h>
#include <fhe/Rot.h>

BOOST_PYTHON_MODULE( fhe )
{
    fhe::PyNode::defineClass();
    fhe::Vec2::defineClass();
    fhe::Vec3::defineClass();
    fhe::Rot2::defineClass();
    fhe::Rot3::defineClass();
}

namespace fhe
{
    PyEnv::PyEnv()
    {
        Py_Initialize();
        
        m_mainModule = boost::python::import("__main__");
        m_mainNamespace = boost::python::dict(m_mainModule.attr("__dict__"));
        m_builtins = m_mainNamespace["__builtins__"].attr("__dict__");
        
        m_mainNamespace["Node"] = PyNode::defineClass();
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
            FHE_ERROR( "error running python file %s", filename.c_str() );
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
            FHE_ERROR( "error execing python script %s", script.c_str() );
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
            FHE_ERROR( "error evaling python script %s", script.c_str() );
        }
    }
    
    std::string PyEnv::getType( boost::python::object obj )                                    
    {                                                                                        
        return boost::python::extract<std::string>(m_builtins["type"](obj).attr("__name__"));
    }                                                                                        
                                                                                                
    Val PyEnv::convert( boost::python::object obj )                                       
    {                                                                                        
        if ( obj == boost::python::object() )                                                
        {                                                                                    
            return Val();                                                                    
        }                                                                                    
        else                                                                                 
        {                                                                                    
            std::string type = getType(obj);                                                 
                                                                                                
            if ( type == "bool" )                                                            
            {                                                                                
                return Val(boost::python::extract<bool>(obj)());                
            }                                                                                
            else if ( type == "int" )                                                        
            {                                                                                
                return Val(boost::python::extract<int>(obj)());                  
            }                                                                                
            else if ( type == "float" )                                                      
            {                                                                                
                return Val(boost::python::extract<double>(obj)());              
            }                                                                                
            else if ( type == "str" )                                                        
            {                                                                                
                return Val(boost::python::extract<std::string>(obj)());  
            }                                                                                
/*                else if ( type == "Vec2" )
            {                                                                                
                return Val::build<Vec2>(boost::python::extract<Vec2>(obj)());                
            }                                                                                
            else if ( type == "Vec3" )                                                       
            {                                                                                
                return Val::build<Vec3>(boost::python::extract<Vec3>(obj)());                
            }                                                                                
            else if ( type == "Rot" )                                                        
            {                                                                                
                return Val::build<Rot>(boost::python::extract<Rot>(obj)());                  
            }                                                                                
            else if ( type == "Quat" )                                                       
            {                                                                                
                return Val::build<Quat>(boost::python::extract<Quat>(obj)());                
            }                                                                                
            else if ( type == "dict" )                                                       
            {                                                                                
                ValMap vm;                                                                   
                boost::python::object items = obj.attr("items")();                           
                for ( int i = 0; i < boost::python::len(items); ++i )                        
                {                                                                            
                    vm._setVal(boost::python::extract<std::string>(items[i][0]),convertToVal(items[i][1]));                                                                                           
                }                                                                                 
                return Val::build<ValMap>(vm);                                                    
            }
            else if ( type == "list" )
            {
                ValList vl;
                for ( int i = 0; i < boost::python::len(obj); ++i )
                {
                    vl.appendRaw( convertToVal(obj[i]) );
                }
                return Val::build<ValList>(vl);
            }*/
            else                                                                                  
            {                                                                                     
                FHE_ERROR( "unable to convert unknown python type %s to var", type.c_str() );
            }                                                                                     
        }                                                                                         
    }                                                                                             
                                                                                                    
    boost::python::object PyEnv::convert( const Val& val )                                   
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
            else if ( val.is<double>() )                                                           
            {                                                                                     
                return boost::python::object(val.get<double>());                                   
            }                                                                                     
            else if ( val.is<std::string>() )                                                     
            {                                                                                     
                return boost::python::object(val.get<std::string>());                             
            }                                                                                     
/*                else if ( val.is<Vec2>() )                                                            
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
            else if ( val.is<ValMap>() )
            {
                boost::python::dict d;
                ValMap vm = val.get<ValMap>();
                std::vector<std::string> names = vm.getValNames();
                for ( std::vector<std::string>::iterator i = names.begin(); i != names.end(); ++i)
                {
                    d[*i] = convertFromVal(vm._getVal(*i));
                }
                return d;
            }
            else if ( val.is<ValList>() )
            {
                boost::python::list l;
                ValList vl = val.get<ValList>();
                for ( int i = 0; i < vl.length(); ++i )
                {
                    l.append(convertFromVal(vl.getRawVal(i)));
                }
                return l;
            }*/
            else
            {
                FHE_ERROR("unable to convert unknown c type to python");
            }
        }
    }
}
