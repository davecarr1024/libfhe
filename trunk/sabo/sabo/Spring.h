#ifndef SABO_SPRING_H
#define SABO_SPRING_H

#include <sabo/PointMass.h>

namespace sabo
{

  class Spring : public boost::noncopyable
  {
    public:
      typedef boost::shared_ptr< Spring > Ptr;
  };

}

#endif
