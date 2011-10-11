#ifndef RE2_ROBOT_DRIVER_DRIVE_H
#define RE2_ROBOT_DRIVER_DRIVE_H

#include <re2robotDriver/DriveConfig.h>
#include <re2robotDriver/DriveCmd.h>
#include <ros/ros.h>

namespace re2
{
    namespace robot
    {
        
        class Driver;
        
        class Drive : public boost::noncopyable
        {
            friend class Driver;
            
            public:
                virtual ~Drive();
                
                const re2robotDriver::DriveConfig& config() const;
                
                virtual bool init();
                virtual void update();
                virtual void shutdown();
                
                virtual double getEffort() const = 0;
                virtual double getVelocity() const = 0;
                virtual double getPosition() const = 0;
                
                virtual void setEffort( double effort ) = 0;
                virtual void setVelocity( double velocity ) = 0;
                virtual void setPosition( double position ) = 0;

            protected:
                Drive();
                
            private:
                ros::NodeHandle m_node;
                re2robotDriver::DriveConfig m_config;
                
                enum Mode
                {
                    MODE_IDLE,
                    MODE_EFFORT,
                    MODE_VELOCITY,
                    MODE_POSITION
                } m_mode;
                
                ros::Time m_lastCmd;
                
                double m_cmd;
                
                ros::Publisher m_configPub;
                
                ros::Publisher m_statePub;
                void sendState();
                
                ros::Subscriber m_cmdSub;
                void cmd( const re2robotDriver::DriveCmd& msg );
                
                ros::Timer m_timer;
                void update( const ros::TimerEvent& e );
                
                void consumeCmd();
                
                bool init( const ros::NodeHandle& node, const re2robotDriver::DriveConfig& config );
        };
        
        typedef boost::shared_ptr< Drive > DrivePtr;
        
    }
}

#endif
