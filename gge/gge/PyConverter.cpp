#include "PyConverter.h"

namespace gge
{
    
    PyConverter::PyConverter()
    {
    }
    
    PyConverter::~PyConverter()
    {
        for ( std::vector<AbstractConverter*>::iterator i = m_fromConverters.begin(); i != m_fromConverters.end(); ++i )
        {
            delete *i;
        }
        m_fromConverters.clear();
        
        for ( std::vector<AbstractConverter*>::iterator i = m_toConverters.begin(); i != m_toConverters.end(); ++i )
        {
            delete *i;
        }
        m_toConverters.clear();
    }
    
    PyConverter& PyConverter::instance()
    {
        static PyConverter instance;
        return instance;
    }
    
    void PyConverter::addToConverter( AbstractConverter* converter )
    {
        assert(converter);
        m_toConverters.push_back(converter);
    }
    
    void PyConverter::addFromConverter( AbstractConverter* converter )
    {
        assert(converter);
        m_fromConverters.push_back(converter);
    }
    
    using namespace boost::python;
    GGE_TO_PYTHON_CONVERTER(object,obj);
}
