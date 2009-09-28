#ifndef PYFUNC_H
#define PYFUNC_H

#include "Func.h"

#include <boost/python.hpp>
#include <cassert>
#include <stdexcept>

namespace fhe
{
    class PyFuncUtil
    {
        private:
            template <class TRet>
            static AbstractFunc* bind2( boost::python::object targ, boost::python::object func );
        
        public:
            static AbstractFunc* bind( boost::python::object tret, boost::python::object targ, boost::python::object func );
    };
    
    template <class TRet, class TArg>
    class PyFunc : public IFunc<TRet, TArg>
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func ) :
                m_func( func )
            {
            }
            
            TRet call( const TArg& arg )
            {
                boost::python::object pyRet;
                try
                {
                    pyRet = m_func(arg);
                }
                catch ( boost::python::error_already_set const& )
                {
                    PyErr_Print();
                    throw std::runtime_error("ERROR: calling pyfunc\n");
                }
                
                return boost::python::extract<TRet>(pyRet);
            }
    };
    
    template<>
    class PyFunc<void,void> : public IFunc<void,void>
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func ) :
                m_func( func )
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
                    throw std::runtime_error("ERROR: calling pyfunc\n");
                }
            }
    };
    template <class TRet>
    class PyFunc<TRet,void> : public IFunc<TRet, void>
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func ) :
                m_func( func )
            {
            }
            
            TRet call()
            {
                boost::python::object pyRet;
                try
                {
                    pyRet = m_func();
                }
                catch ( boost::python::error_already_set const& )
                {
                    PyErr_Print();
                    throw std::runtime_error("ERROR: calling pyfunc\n");
                }
                
                return boost::python::extract<TRet>(pyRet);
            }
    };
    
    template <class TArg>
    class PyFunc<void,TArg> : public IFunc<void, TArg>
    {
        private:
            boost::python::object m_func;
            
        public:
            PyFunc( boost::python::object func ) :
                m_func( func )
            {
            }
            
            void call( const TArg& arg )
            {
                try
                {
                    m_func(arg);
                }
                catch ( boost::python::error_already_set const& )
                {
                    PyErr_Print();
                    throw std::runtime_error("ERROR: calling pyfunc\n");
                }
            }
    };
}

#endif
