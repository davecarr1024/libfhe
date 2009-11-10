#ifndef FUNC_H
#define FUNC_H

#include "Var.h"

#include <string>

namespace fhe
{
    
    template <class T>
    class Func;
    
    class AbstractFunc
    {
        public:
            virtual std::string getName()=0;
            
            virtual Var call( const Var& arg )=0;
    };
    
    template <class T>
    class Func : public AbstractFunc
    {
        public:
            typedef Var (T::*Method)( const Var&);
            
        private:
            std::string m_name;
            T* m_obj;
            Method m_method;
            
        public:
            Func( const std::string& name, T* obj, Method method ) :
                m_name(name),
                m_obj(obj),
                m_method(method)
            {
                assert(m_obj);
                assert(m_method);
            }
            
            const std::type_info& getType()
            {
                return typeid(T);
            }
            
            std::string getName()
            {
                return m_name;
            }
            
            Var call( const Var& arg )
            {
                return (m_obj->*m_method)(arg);
            }
    };
    
}

#endif
