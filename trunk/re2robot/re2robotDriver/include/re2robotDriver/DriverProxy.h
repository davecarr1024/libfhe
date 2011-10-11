#ifndef RE2_ROBOT_DRIVER_DRIVER_PROXY_H
#define RE2_ROBOT_DRIVER_DRIVER_PROXY_H

#include <re2robotDriver/DriveProxy.h>
#include <re2robotDriver/DriverConfig.h>

namespace re2
{
    namespace robot
    {
        
        class DriverProxy : public boost::noncopyable
        {
            public:
                typedef boost::function< void( const std::vector< DriveProxyPtr >& drivesAdded, const std::vector< DriveProxyPtr >& drivesRemoved ) > ChangeCB;

                DriverProxy( const ros::NodeHandle& node, const ChangeCB& changeCB = ChangeCB() );
                virtual ~DriverProxy();
                
                DriveProxyPtr addDrive( const re2robotDriver::DriveConfig& config );
                bool removeDrive( const std::string& name );
                DriveProxyPtr getDrive( const std::string& name ) const;
                
                typedef std::map< std::string, DriveProxyPtr >::const_iterator DriveIterator;
                DriveIterator drivesBegin() const;
                DriveIterator drivesEnd() const;
                
                void setChangeCB( const ChangeCB& changeCB );
                
            private:
                ros::NodeHandle m_node;
                std::map< std::string, DriveProxyPtr > m_drives;
                
                ros::Subscriber m_configSub;
                void config( const re2robotDriver::DriverConfig& msg );
                
                ros::ServiceClient m_addDriveClient;
                
                ros::ServiceClient m_removeDriveClient;
                
                ChangeCB m_changeCB;
        };
        
    }
}

#endif
