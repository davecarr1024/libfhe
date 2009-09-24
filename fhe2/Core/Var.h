#ifndef VAR_H
#define VAR_H

#include <string>
#include <boost/python.hpp>

namespace fhe
{
    
    class VarMap;
    
    class Var
    {
        public:
            enum Type
            {
                NONE,
                BOOL,
                INT,
                FLOAT,
                STRING,
                VARMAP
            };
            
        private:
            Type m_type;
            
            union Data
            {
                bool b;
                int i;
                float f;
                std::string* s;
                VarMap* vm;
            };
            
            Data m_data;
            
            void setType( Type type );
            
        public:
            Var();
            Var( const Var& var );
            Var& operator=( const Var& var );
            Var( boost::python::object obj );
            
            ~Var();
            
            Type getType();
            
            template <class T>
            bool is();
            
            template <class T>
            T get();
            
            template <class T>
            T get(const T& def);
            
            template <class T>
            void set(const T& val);
            
            boost::python::object pyGet();
            
            void pySet( boost::python::object val );
            
            boost::python::object toPy();
            
            static Var fromPy( boost::python::object obj );
            
            static boost::python::object defineClass();
    };
    
}

#endif
