#ifndef RE2_ROBOT_MODEL_SIMPLE_TRANSMISSION_H
#define RE2_ROBOT_MODEL_SIMPLE_TRANSMISSION_H

#include <re2robotModel/Transmission.h>
#include <Poco/MetaObject.h>

namespace re2
{
    namespace robot
    {
        
        class SimpleTransmission : public Transmission
        {
            typedef Transmission Super;
            friend class Poco::MetaObject< SimpleTransmission, Transmission >;
            
            public:
                virtual ~SimpleTransmission();
                
                double getEffort() const;
                double getVelocity() const;
                double getPosition() const;
                
                void setEffort( double effort );
                void setVelocity( double velocity );
                void setPosition( double position );
                
            private:
                SimpleTransmission();
        };
        
    }
}

#endif
