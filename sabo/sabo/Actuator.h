#ifndef SABO_ACTUATOR_H
#define SABO_ACTUATOR_H

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

namespace sabo
{

  class Body;

  class Actuator : public boost::noncopyable
  {
    public:
      typedef boost::shared_ptr< Actuator > Ptr;
    
      Actuator( Body* body );
      virtual ~Actuator();
      
      Body* body() const;
      
      virtual void update( double dt ) = 0;
      
    private:
      Body* m_body;
  };

}

#endif
