#ifndef FUNC_H
#define FUNC_H

#include <boost/python.hpp>
#include <cassert>

#include "PyConverter.h"

namespace fhe
{
    
    template <class TRet, class TArg>
    class IFunc;
    
    class AbstractFunc
    {
        public:
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
            
            boost::python::object pyCall( boost::python::object arg )
            {
                return PyConverter::toPy<TRet>(call(PyConverter::fromPy<TArg>(arg)));
            }
    };
    
    template <class TRet>
    class IFunc<TRet,void> : public AbstractFunc
    {
        public:
            virtual TRet call()=0;

            boost::python::object pyCall( boost::python::object arg )
            {
                return PyConverter::toPy<TRet>(call());
            }
    };
    
    template <class TArg>
    class IFunc<void,TArg> : public AbstractFunc
    {
        public:
            virtual void call( const TArg& arg )=0;
            
            boost::python::object pyCall( boost::python::object arg )
            {
                call(PyConverter::fromPy<TArg>(arg));
                return boost::python::object();
            }
    };
    
    template <>
    class IFunc<void,void> : public AbstractFunc
    {
        public:
            virtual void call()=0;
            
            boost::python::object pyCall( boost::python::object arg )
            {
                call();
                return boost::python::object();
            }
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
                assert(m_obj);
                assert(m_method);
            }
            
            TRet call( const TArg& arg )
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
                assert(m_obj);
                assert(m_method);
            }
            
            TRet call()
            {
                return (m_obj->*m_method)();
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
