#ifndef RE2_ROBOT_DRIVER_DRIVE_PROXY_H
#define RE2_ROBOT_DRIVER_DRIVE_PROXY_H

#include <re2robotDriver/DriveConfig.h>
#include <re2robotDriver/DriveState.h>
#include <ros/ros.h>

namespace re2
{
    namespace robot
    {
        
        class DriverProxy;
        
        class DriveProxy : public boost::noncopyable
        {
            friend class DriverProxy;
            
            public:
                virtual ~DriveProxy();
                
                const re2robotDriver::DriveConfig& config() const;
                const re2robotDriver::DriveState& state() const;
                
                void setEffort( double effort );
                void setVelocity( double velocity );
                void setPosition( double position );
                
            private:
                DriveProxy( const ros::NodeHandle& node, const re2robotDriver::DriveConfig& config );
                
                ros::NodeHandle m_node;
                const re2robotDriver::DriveConfig m_config;
                re2robotDriver::DriveState m_state;
                
                ros::Subscriber m_stateSub;
                void state( const re2robotDriver::DriveState& msg );
                
                ros::Publisher m_cmdPub;
        };
        
        typedef boost::shared_ptr< DriveProxy > DriveProxyPtr;
    }
}

#endif
