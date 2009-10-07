#ifndef PYCONVERTER_H
#define PYCONVERTER_H

#include <boost/python.hpp>

namespace fhe
{
    class PyConverter
    {
        public:
            template <class T>
            static T fromPy( boost::python::object obj );
            
            template <class T>
            static boost::python::object toPy( const T& obj );
    };
    
}

#endif
