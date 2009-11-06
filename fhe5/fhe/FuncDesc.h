#ifndef FUNC_DESC_H
#define FUNC_DESC_H

#include "Func.h"

namespace fhe
{
    
    template <class T>
    class FuncDesc;
    
    class AbstractFuncDesc
    {
        public:
            virtual const std::type_info& getType()=0;
            
            virtual std::string getName()=0;
            
            template <class T>
            bool is()
            {
                return typeid(T) == getType();
            }
            
            template <class T>
            FuncDesc<T>* cast()
            {
                return is<T>() ? static_cast<FuncDesc<T>*>(this) : 0;
            }
    };
    
    template <class T>
    class FuncDesc : public AbstractFuncDesc
    {
        public:
            typedef Var (T::*Method)( const Var&);
            
        private:
            std::string m_name;
            Method m_method;
            
        public:
            FuncDesc( const std::string& name, Method method ) :
                m_method(method),
                m_name(name)
            {
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
            
            Func<T>* instantiate( T* obj )
            {
                return new Func<T>(m_name,obj,m_method);
            }
    };
    
}

#endif
