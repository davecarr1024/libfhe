#ifndef RE2_ROBOT_DRIVER_SIM_DRIVE_H
#define RE2_ROBOT_DRIVER_SIM_DRIVE_H

#include <re2robotDriver/Drive.h>
#include <Poco/MetaObject.h>

namespace re2
{
    namespace robot
    {
        
        class SimDrive : public Drive
        {
            typedef Drive Super;
            friend class Poco::MetaObject< SimDrive, Drive >;
            
            public:
                virtual ~SimDrive();
                
                bool init();
                void update();
                
                double getEffort() const;
                double getVelocity() const;
                double getPosition() const;
                
                void setEffort( double effort );
                void setVelocity( double velocity );
                void setPosition( double position );
                
            protected:
                SimDrive();
                
            private:
                double m_position;
                double m_velocity;
                
                enum Mode
                {
                    MODE_EFFORT,
                    MODE_VELOCITY,
                    MODE_POSITION
                } m_mode;
                
                double m_cmd;
                
                ros::Time m_lastUpdate;
        };
        
    }
}

#endif
