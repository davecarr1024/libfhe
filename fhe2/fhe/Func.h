#ifndef FUNC_H
#define FUNC_H

#include <boost/python.hpp>
#include <boost/function.hpp>
#include <cassert>

namespace fhe
{
    template <class TRet, class TArg>
    class IFunc;
    
    class AbstractFunc
    {
        public:
            virtual bool pyCanCall( boost::python::object arg )=0;
            virtual boost::python::object pyCall( boost::python::object arg )=0;

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
            
            bool pyCanCall( boost::python::object arg )
            {
                return boost::python::extract<TArg&>(arg).check();
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                assert(pyCanCall(arg));
                return boost::python::object(call(boost::python::extract<TArg>(arg)));
            }
    };
    
    template <class TRet>
    class IFunc<TRet,void> : public AbstractFunc
    {
        public:
            virtual TRet call()=0;

            bool pyCanCall( boost::python::object arg )
            {
                return arg == boost::python::object();
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                assert(pyCanCall(arg));
                return boost::python::object(call());
            }
    };
    
    template <class TArg>
    class IFunc<void,TArg> : public AbstractFunc
    {
        public:
            virtual void call( const TArg& arg )=0;
            
            bool pyCanCall( boost::python::object arg )
            {
                return boost::python::extract<TArg&>(arg).check();
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                assert(pyCanCall(arg));
                call(boost::python::extract<TArg>(arg));
                return boost::python::object();
            }
    };
    
    template <>
    class IFunc<void,void> : public AbstractFunc
    {
        public:
            virtual void call()=0;
            
            bool pyCanCall( boost::python::object arg )
            {
                return arg == boost::python::object();
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                assert(pyCanCall(arg));
                call();
                return boost::python::object();
            }
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
    
    template <class TRet, class TArg>
    class PyFunc : public IFunc<TRet,TArg>
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func ) :
                m_func(func)
            {
            }
            
            TRet call( const TArg& arg )
            {
                try
                {
                    return boost::python::extract<TRet>(m_func(boost::python::object(arg)));
                }
                catch ( boost::python::error_already_set const& )
                {
                    PyErr_Print();
                    throw std::runtime_error("error calling python function");
                }
            }
    };
    
    template <class TRet>
    class PyFunc<TRet,void> : public IFunc<TRet,void>
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func ) :
                m_func(func)
            {
            }
            
            TRet call()
            {
                try
                {
                    return boost::python::extract<TRet>(m_func());
                }
                catch ( boost::python::error_already_set const& )
                {
                    PyErr_Print();
                    throw std::runtime_error("error calling python function");
                }
            }
    };
    
    template <class TArg>
    class PyFunc<void,TArg> : public IFunc<void,TArg>
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func ) :
                m_func(func)
            {
            }
            
            void call( const TArg& arg )
            {
                try
                {
                    m_func(boost::python::object(arg));
                }
                catch ( boost::python::error_already_set const& )
                {
                    PyErr_Print();
                    throw std::runtime_error("error calling python function");
                }
            }
    };
    
    template <>
    class PyFunc<void,void> : public IFunc<void,void>
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func ) :
                m_func(func)
            {
            }
            
            void call()
            {
                try
                {
                    m_func();
                }
                catch ( boost::python::error_already_set const& )
                {
                    PyErr_Print();
                    throw std::runtime_error("error calling python function");
                }
            }
    };
}

#endif
