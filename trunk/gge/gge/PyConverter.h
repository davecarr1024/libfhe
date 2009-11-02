#ifndef PYCONVERTER_H
#define PYCONVERTER_H

#include <boost/python.hpp>

#include <typeinfo>
#include <vector>
#include <cassert>

namespace gge
{
    
    class AbstractConverter
    {
        private:
            std::string m_name;
        
        public:
            AbstractConverter( const std::string& name ) :
                m_name(name)
            {
            }
            
            std::string getName()
            {
                return m_name;
            }
            
            virtual const std::type_info& getType()=0;
            
            template <class T>
            bool is()
            {
                return typeid(T) == getType();
            }
    };
    
    template <class T>
    class IFromConverter : public AbstractConverter
    {
        public:
            IFromConverter( const std::string& name ) : AbstractConverter(name) {}
            
            const std::type_info& getType()
            {
                return typeid(T);
            }
            
            virtual T operator()( boost::python::object obj )=0;
    };
    
    template <class T>
    class IToConverter : public AbstractConverter
    {
        public:
            IToConverter( const std::string& name ) : AbstractConverter(name) {}
            
            const std::type_info& getType()
            {
                return typeid(T);
            }
            
            virtual boost::python::object operator()( const T& t )=0;
    };
    
    class PyConverter
    {
        private:
            std::vector<AbstractConverter*> m_fromConverters, m_toConverters;
            
            PyConverter();
            
        public:
            ~PyConverter();
            
            static PyConverter& instance();
            
            void addToConverter( AbstractConverter* converter );
            
            void addFromConverter( AbstractConverter* converter );

            template <class T>
            IFromConverter<T>* getFromConverter()
            {
                for ( std::vector<AbstractConverter*>::iterator i = m_fromConverters.begin(); i != m_fromConverters.end(); ++i )
                {
                    if ( (*i)->is<T>() )
                    {
                        return static_cast<IFromConverter<T>*>(*i);
                    }
                }
                return 0;
            }
            
            template <class T>
            IToConverter<T>* getToConverter()
            {
                for ( std::vector<AbstractConverter*>::iterator i = m_toConverters.begin(); i != m_toConverters.end(); ++i )
                {
                    if ( (*i)->is<T>() )
                    {
                        return static_cast<IToConverter<T>*>(*i);
                    }
                }
                return 0;
            }
            
            template <class T>
            boost::python::object toPy( const T& t )
            {
                IToConverter<T>* converter = getToConverter<T>();
                return converter ? (*converter)(t) : boost::python::object(t);
            }
            
            template <class T>
            T fromPy( boost::python::object obj )
            {
                IFromConverter<T>* converter = getFromConverter<T>();
                return converter ? (*converter)(obj) : boost::python::extract<T>(obj)();
            }
    };
    
    class ToConverterRegisterer
    {
        public:
            ToConverterRegisterer( AbstractConverter* converter )
            {
                PyConverter::instance().addToConverter(converter);
            }
    };

    class FromConverterRegisterer
    {
        public:
            FromConverterRegisterer( AbstractConverter* converter )
            {
                PyConverter::instance().addFromConverter(converter);
            }
    };

    #define GGE_FROM_PYTHON_CONVERTER(type,code) \
        class type##_FromConverter : public IFromConverter<type> { \
        public:\
            type##_FromConverter( const std::string& name ) : IFromConverter<type>(name) {} \
            type operator()( boost::python::object obj ) { return code; } \
        }; \
        FromConverterRegisterer type##_fromregisterer( new type##_FromConverter(#type) );
        
    #define GGE_TO_PYTHON_CONVERTER(type,code) \
        class type##_ToConverter : public IToConverter<type> { \
        public:\
            type##_ToConverter( const std::string& name ) : IToConverter<type>(name) {} \
            boost::python::object operator()( const type& obj ) { return code; } \
        }; \
        ToConverterRegisterer type##_toregisterer( new type##_ToConverter(#type) );
    
}

#endif
