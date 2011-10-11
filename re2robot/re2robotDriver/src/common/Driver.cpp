#include <re2robotDriver/Driver.h>
#include <re2robotDriver/DriverConfig.h>
#include <re2robotUtil/Config.h>

namespace re2
{
    namespace robot
    {
        
        Driver::Driver( const ros::NodeHandle& node, const std::string& robotDescPath ) :
            m_node( node ),
            m_configPub( m_node.advertise< re2robotDriver::DriverConfig >( "/re2/robot/driver/config", 1, true ) ),
            m_addDriveSrv( m_node.advertiseService( "/re2/robot/driver/add_drive", &Driver::addDrive, this ) ),
            m_removeDriveSrv( m_node.advertiseService( "/re2/robot/driver/remove_drive", &Driver::removeDrive, this ) ),
            m_driveLoader( "re2robotDriver", "re2::robot::Drive" )
        {
            if ( !robotDescPath.empty() )
            {
                Config config( robotDescPath ),
                       robot( config["robot"] ),
                       driveConfigs( robot["drive"] );
                for ( size_t i = 0; i < driveConfigs.size(); ++i )
                {
                    Config driveConfigIn( driveConfigs[i] );
                    re2robotDriver::DriveConfig driveConfigOut;
                    ROS_ASSERT_MSG( driveConfigIn[ "name" ].tryTo< std::string >( driveConfigOut.name ),
                                    "unable to load name for drive from robot desc %s", robotDescPath.c_str() );
                    ROS_ASSERT_MSG( driveConfigIn[ "type" ].tryTo< std::string >( driveConfigOut.type ),
                                    "unable to get type for drive %s from robot desc %s", driveConfigOut.name.c_str(), robotDescPath.c_str() );
                    driveConfigOut.updateIntervalS = driveConfigIn[ "updateIntervalS" ].to< double >( 0.01 );
                    driveConfigOut.cmdTimeoutS = driveConfigIn[ "cmdTimeoutS" ].to< double >( 0.1 );
                    driveConfigOut.maxVelocity = driveConfigIn[ "maxVelocity" ].to< double >( 10 );
                    driveConfigOut.minPosition = driveConfigIn[ "minPosition" ].to< double >( -100 );
                    driveConfigOut.maxPosition = driveConfigIn[ "maxPosition" ].to< double >( 100 );
                    driveConfigOut.initialPosition = driveConfigIn[ "initialPosition" ].to< double >( 0 );
                    driveConfigOut.positionPGain = driveConfigIn[ "positionPGain" ].to< double >( 3 );
                    driveConfigOut.positionDGain = driveConfigIn[ "positionDGain" ].to< double >( 0.1 );
                    driveConfigOut.velocityTimeToTargetS = driveConfigIn[ "velocityTimeToTargetS" ].to< double >( 0.1 );
                    ROS_ASSERT_MSG( addDrive( driveConfigOut ), "unable to add drive %s from robot desc %s", driveConfigOut.name.c_str(), robotDescPath.c_str() );
                }
            }
            
            sendConfig();
        }
        
        Driver::~Driver()
        {
        }
        
        DrivePtr Driver::addDrive( const re2robotDriver::DriveConfig& config )
        {
            if ( m_drives.find( config.name ) != m_drives.end() )
            {
                ROS_ERROR( "trying to add duplicate drive %s", config.name.c_str() );
                return DrivePtr();
            }
            else
            {
                DrivePtr drive;
                try
                {
                    drive.reset( m_driveLoader.createClassInstance( config.type ) );
                }
                catch ( pluginlib::PluginlibException& e )
                {
                    ROS_ERROR( "unable to load drive type %s: %s", config.type.c_str(), e.what() );
                    return DrivePtr();
                }
                if ( !drive->init( m_node, config ) )
                {
                    ROS_ERROR( "failed to init drive %s of type %s", drive->config().name.c_str(), drive->config().type.c_str() );
                    return DrivePtr();
                }
                else
                {
                    m_drives[ drive->config().name ] = drive;
                    sendConfig();
                    return drive;
                }
            }
        }
        
        bool Driver::removeDrive( const std::string& name )
        {
            std::map< std::string, DrivePtr >::iterator i = m_drives.find( name );
            if ( i == m_drives.end() )
            {
                ROS_ERROR( "trying to remove unknown drive %s", name.c_str() );
                return false;
            }
            else
            {
                i->second->shutdown();
                m_drives.erase( i );
                sendConfig();
                return true;
            }
        }
        
        DrivePtr Driver::getDrive( const std::string& name ) const
        {
            std::map< std::string, DrivePtr >::const_iterator i = m_drives.find( name );
            if ( i == m_drives.end() )
            {
                return DrivePtr();
            }
            else
            {
                return i->second;
            }
        }
        
        Driver::DriveIterator Driver::drivesBegin() const
        {
            return m_drives.begin();
        }
        
        Driver::DriveIterator Driver::drivesEnd() const
        {
            return m_drives.end();
        }
        
        bool Driver::addDrive( re2robotDriver::AddDrive::Request& req,
                               re2robotDriver::AddDrive::Response& res )
        {
            res.success = bool( addDrive( req.config ) );
            return true;
        }
        
        bool Driver::removeDrive( re2robotDriver::RemoveDrive::Request& req,
                                  re2robotDriver::RemoveDrive::Response& res )
        {
            res.success = removeDrive( req.name );
            return true;
        }
        
        void Driver::sendConfig()
        {
            re2robotDriver::DriverConfig msg;
            for ( std::map< std::string, DrivePtr >::const_iterator i = m_drives.begin(); i != m_drives.end(); ++i )
            {
                msg.drives.push_back( i->second->config() );
            }
            m_configPub.publish( msg );
        }
        
    }
}
