#ifndef FUNC_H
#define FUNC_H

#include "PyConverter.h"

#include <typeinfo>
#include <boost/python.hpp>

namespace gge
{
    template <class TRet, class TArg>
    class IFunc;
    
    class AbstractFunc
    {
        public:
            virtual const std::type_info& getRetType()=0;
            virtual const std::type_info& getArgType()=0;
            virtual boost::python::object pyCall( boost::python::object arg )=0;
            
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
            
            boost::python::object pyCall( boost::python::object arg )
            {
                return PyConverter::instance().toPy<TRet>((*this)(PyConverter::instance().fromPy<TArg>(arg)));
            }
            
            virtual TRet operator()( const TArg& arg )=0;
    };
    
    template <class TArg>
    class IFunc<void,TArg> : public AbstractFunc
    {
        public:
            const std::type_info& getRetType()
            {
                return typeid(void);
            }
            
            const std::type_info& getArgType()
            {
                return typeid(TArg);
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                (*this)(PyConverter::instance().fromPy<TArg>(arg));
                return boost::python::object();
            }
            
            virtual void operator()( const TArg& arg )=0;
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
            
            boost::python::object pyCall( boost::python::object arg )
            {
                return PyConverter::instance().toPy<TRet>((*this)());
            }
            
            virtual TRet operator()()=0;
    };
    
    template <>
    class IFunc<void,void> : public AbstractFunc
    {
        public:
            const std::type_info& getRetType()
            {
                return typeid(void);
            }
            
            const std::type_info& getArgType()
            {
                return typeid(void);
            }
            
            boost::python::object pyCall( boost::python::object arg )
            {
                (*this)();
                return boost::python::object();
            }
            
            virtual void operator()()=0;
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
            
            TRet operator()( const TArg& arg )
            {
                return PyConverter::instance().fromPy<TRet>(m_func(PyConverter::instance().toPy<TArg>(arg)));
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
            
            TRet operator()()
            {
                return PyConverter::instance().fromPy<TRet>(m_func());
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
            
            void operator()( const TArg& arg )
            {
                m_func(PyConverter::instance().toPy<TArg>(arg));
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
            
            void operator()()
            {
                m_func();
            }
    };
    
}

#endif
