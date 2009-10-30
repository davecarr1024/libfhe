#ifndef FUNC_H
#define FUNC_H

#include <typeinfo>

namespace gge
{
    template <class TRet, class TArg>
    class IFunc;
    
    class AbstractFunc
    {
        public:
            virtual const std::type_info& getRetType()=0;
            virtual const std::type_info& getArgType()=0;
            
            template <class TRet, class TArg>
            bool is()
            {
                return typeid(TRet) == getRetType() && typeid(TArg) == getArgType();
            }
            
            template <class TRet, class TArg>
            IFunc<TRet,TArg>* cast()
            {
                return is<TRet,TArg>() ? static_cast<IFunc<TRet,TArg>*>(this) : 0;
            }
    };
    
    template <class TRet, class TArg>
    class IFunc : public AbstractFunc
    {
        public:
            const std::type_info& getRetType()
            {
                return typeid(TRet);
            }
            
            const std::type_info& getArgType()
            {
                return typeid(TArg);
            }
            
            virtual TRet operator()( const TArg& arg )=0;
    };
    
    template <class TRet>
    class IFunc<TRet,void> : public AbstractFunc
    {
        public:
            const std::type_info& getRetType()
            {
                return typeid(TRet);
            }
            
            const std::type_info& getArgType()
            {
                return typeid(void);
            }
            
            virtual TRet operator()()=0;
    };
    
    template <class TObj, class TRet, class TArg>
    class Func : public IFunc<TRet,TArg>
    {
        public:
            typedef TRet (TObj::*Method)(TArg);
            
        private:
            TObj* m_obj;
            Method m_method;
            
        public:
            Func( TObj* obj, Method method ) :
                m_obj(obj),
                m_method(method)
            {
            }
            
            TRet operator()( const TArg& arg )
            {
                return (m_obj->*m_method)(arg);
            }
    };
    
    template <class TObj, class TRet>
    class Func<TObj,TRet,void> : public IFunc<TRet,void>
    {
        public:
            typedef TRet (TObj::*Method)();
            
        private:
            TObj* m_obj;
            Method m_method;
            
        public:
            Func( TObj* obj, Method method ) :
                m_obj(obj),
                m_method(method)
            {
            }
            
            TRet operator()()
            {
                return (m_obj->*m_method)();
            }
    };
}

#endif
