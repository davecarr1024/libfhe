#ifndef VAR_H
#define VAR_H

#include <string>
#include <boost/python.hpp>

namespace fhe
{
    class Vec2;
    class Rot;
    class Mat3;
    class Vec3;
    class Quat;
    class Mat4;
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
                VEC2,
                ROT,
                MAT3,
                VEC3,
                QUAT,
                MAT4,
                VARMAP
            };
            
        private:
            Type m_type;
            
            union VarData
            {
                bool b;
                int i;
                float f;
                std::string* s;
                Vec2* v2;
                Rot* r;
                Mat3* m3;
                Vec3* v3;
                Quat* q;
                Mat4* m4;
                VarMap* vm;
            };
            
            VarData m_data;
            
            void setType( Type type );
            
        public:
            Var();
            
            ~Var();
            
            Var( const Var& var );
            
            Var& operator=( const Var& var );
            
            Type getType();
            
            template <class T>
            void set( const T& val );
            
            template <class T>
            T get();
            
            template <class T>
            bool is();
            
            boost::python::object toPy();
            
            static Var fromPy( boost::python::object obj );
    };
    
}

#endif
