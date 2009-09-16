#ifndef FUNC_H
#define FUNC_H

#include <boost/python.hpp>
#include <boost/function.hpp>
#include <cassert>

namespace fhe
{
    
    template <class TRet, class TArg>
    class IFunc;
    
    class IFuncWrapper
    {
        public:
            virtual ~IFuncWrapper() {}
            
            template <class TRet, class TArg>
            IFunc<TRet,TArg>* cast()
            {
                return dynamic_cast<IFunc<TRet,TArg>*>(this);
            }
            
            virtual bool pyCanCall( boost::python::object arg )=0;
            virtual boost::python::object pyCall( boost::python::object arg )=0;
    };
    
    template <class TRet, class TArg>
    class IFunc : public IFuncWrapper
    {
        public:
            virtual TRet call( const TArg& arg )=0;

            bool pyCanCall( boost::python::object arg )
            {
                return boost::python::extract<TArg>(arg).check();
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                assert( pyCanCall( arg ) );
                return boost::python::object( call( boost::python::extract<TArg>(arg) ) );
            }
    };
    
    template <>
    class IFunc<void,void> : public IFuncWrapper
    {
        public:
            virtual void call()=0;

            bool pyCanCall( boost::python::object arg )
            {
                return arg == boost::python::object();
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                assert( pyCanCall( arg ) );
                call();
                return boost::python::object();
            }
    };
    
    template <class TArg>
    class IFunc<void,TArg> : public IFuncWrapper
    {
        public:
            virtual void call( const TArg& arg )=0;

            bool pyCanCall( boost::python::object arg )
            {
                return boost::python::extract<TArg>(arg).check();
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                assert( pyCanCall( arg ) );
                call( boost::python::extract<TArg>(arg) );
                return boost::python::object();
            }
    };
    
    template <class TRet>
    class IFunc<TRet,void> : public IFuncWrapper
    {
        public:
            virtual TRet call()=0;
            
            bool pyCanCall( boost::python::object arg )
            {
                return arg == boost::python::object();
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                assert( pyCanCall( arg ) );
                return boost::python::object( call() );
            }
    };
    
    template <class TRet, class TArg>
    class Func : public IFunc<TRet,TArg>
    {
        public:
            typedef boost::function<TRet (TArg)> FuncPtr;
            
        private:
            FuncPtr m_func;
            
        public:
            Func( FuncPtr func ) :
                m_func( func )
            {
            }
            
            TRet call( const TArg& arg )
            {
                assert( m_func );
                return m_func( arg );
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
            }
            
            TRet call()
            {
                assert( m_func );
                return m_func();
            }
    };
}

#endif
