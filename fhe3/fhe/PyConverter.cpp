#include "PyConverter.h"
#include "PyConverters.h"

namespace fhe
{
    template <class T>
    T PyConverter::fromPy<T>( boost::python::object obj )
    {
        return FromPyConverter<T>()(obj);
    }
    
    template <class T>
    boost::python::object PyConverter::toPy<T>( const T& obj )
    {
        return ToPyConverter<T>()(obj);
    }
}
