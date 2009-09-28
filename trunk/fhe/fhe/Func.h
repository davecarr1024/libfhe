#ifndef FUNC_H
#define FUNC_H

#include <boost/function.hpp>
#include <cassert>

namespace fhe
{
    template <class TRet, class TArg>
    class IFunc;
    
    class AbstractFunc
    {
        public:
            virtual ~AbstractFunc() {}
            
            template <class TRet, class TArg>
            IFunc<TRet,TArg>* cast()
            {
                return dynamic_cast<IFunc<TRet,TArg>*>(this);
            }
    };
    
    template <class TRet, class TArg>
    class IFunc : public AbstractFunc
    {
        public:
            virtual TRet call( const TArg& )=0;
    };
    
    template <class TRet>
    class IFunc<TRet,void> : public AbstractFunc
    {
        public:
            virtual TRet call()=0;
    };
    
    template <class TRet, class TArg>
    class Func : public IFunc<TRet,TArg>
    {
        public:
            typedef boost::function<TRet(TArg)> FuncPtr;
            
        private:
            FuncPtr m_func;
            
        public:
            Func( FuncPtr func ) :
                m_func(func)
            {
                assert(m_func);
            }
            
            TRet call( const TArg& arg )
            {
                return m_func(arg);
            }
    };
    
    template <class TRet>
    class Func<TRet,void> : public IFunc<TRet,void>
    {
        public:
            typedef boost::function<TRet(void)> FuncPtr;
            
        private:
            FuncPtr m_func;
            
        public:
            Func( FuncPtr func ) :
                m_func( func )
            {
                assert(m_func);
            }
            
            TRet call()
            {
                return m_func();
            }
    };
}

#endif
