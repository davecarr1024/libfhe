#ifndef RE2_ROBOT_DRIVER_DRIVER_H
#define RE2_ROBOT_DRIVER_DRIVER_H

#include <re2robotDriver/Drive.h>
#include <re2robotDriver/AddDrive.h>
#include <re2robotDriver/RemoveDrive.h>
#include <pluginlib/class_loader.h>

namespace re2
{
    namespace robot
    {
        
        class Driver : public boost::noncopyable
        {
            public:
                Driver( const ros::NodeHandle& node, const std::string& robotDescPath = "/robot_description" );
                virtual ~Driver();
                
                DrivePtr addDrive( const re2robotDriver::DriveConfig& config );
                bool removeDrive( const std::string& name );
                DrivePtr getDrive( const std::string& name ) const;
                
                typedef std::map< std::string, DrivePtr >::const_iterator DriveIterator;
                DriveIterator drivesBegin() const;
                DriveIterator drivesEnd() const;
                
            private:
                ros::NodeHandle m_node;
                std::map< std::string, DrivePtr > m_drives;
                
                ros::Publisher m_configPub;
                void sendConfig();
                
                ros::ServiceServer m_addDriveSrv;
                bool addDrive( re2robotDriver::AddDrive::Request& req,
                               re2robotDriver::AddDrive::Response& res );
                               
                ros::ServiceServer m_removeDriveSrv;
                bool removeDrive( re2robotDriver::RemoveDrive::Request& req,
                                  re2robotDriver::RemoveDrive::Response& res );
                                  
                pluginlib::ClassLoader< Drive > m_driveLoader;
        };
        
    }
}

#endif
