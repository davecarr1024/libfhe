#include <re2robotDriver/DriverProxy.h>
#include <re2robotDriver/AddDrive.h>
#include <re2robotDriver/RemoveDrive.h>

namespace re2
{
    namespace robot
    {
        
        DriverProxy::DriverProxy( const ros::NodeHandle& node, const ChangeCB& changeCB ) :
            m_node( node ),
            m_configSub( m_node.subscribe( "/re2/robot/driver/config", 1, &DriverProxy::config, this ) ),
            m_addDriveClient( m_node.serviceClient< re2robotDriver::AddDrive >( "/re2/robot/driver/add_drive" ) ),
            m_removeDriveClient( m_node.serviceClient< re2robotDriver::RemoveDrive >( "/re2/robot/driver/remove_drive" ) ),
            m_changeCB( changeCB )
        {
        }
        
        DriverProxy::~DriverProxy()
        {
        }
        
        DriveProxyPtr DriverProxy::addDrive( const re2robotDriver::DriveConfig& config )
        {
            re2robotDriver::AddDrive srv;
            srv.request.config = config;
            if ( m_addDriveClient.call( srv ) && srv.response.success )
            {
                DriveProxyPtr drive( new DriveProxy( m_node, config ) );
                m_drives[ drive->config().name ] = drive;
                
                if ( m_changeCB )
                {
                    std::vector< DriveProxyPtr > drivesAdded, drivesRemoved;
                    drivesAdded.push_back( drive );
                    m_changeCB( drivesAdded, drivesRemoved );
                }
                
                return drive;
            }
            else
            {
                return DriveProxyPtr();
            }
        }
        
        bool DriverProxy::removeDrive( const std::string& name )
        {
            re2robotDriver::RemoveDrive srv;
            srv.request.name = name;
            if ( m_removeDriveClient.call( srv ) && srv.response.success )
            {
                std::map< std::string, DriveProxyPtr >::iterator i = m_drives.find( name );
                if ( i != m_drives.end() )
                {
                    m_drives.erase( i );
                    
                    if ( m_changeCB )
                    {
                        std::vector< DriveProxyPtr > drivesAdded, drivesRemoved;
                        drivesRemoved.push_back( i->second );
                        m_changeCB( drivesAdded, drivesRemoved );
                    }
                }
                return true;
            }
            else
            {
                return false;
            }
        }
        
        DriveProxyPtr DriverProxy::getDrive( const std::string& name ) const
        {
            std::map< std::string, DriveProxyPtr >::const_iterator i = m_drives.find( name );
            if ( i == m_drives.end() )
            {
                return DriveProxyPtr();
            }
            else
            {
                return i->second;
            }
        }
        
        DriverProxy::DriveIterator DriverProxy::drivesBegin() const
        {
            return m_drives.begin();
        }
        
        DriverProxy::DriveIterator DriverProxy::drivesEnd() const
        {
            return m_drives.end();
        }
        
        void DriverProxy::config( const re2robotDriver::DriverConfig& msg )
        {
            std::vector< DriveProxyPtr > drivesAdded, drivesRemoved;
            for ( std::map< std::string, DriveProxyPtr >::const_iterator i = m_drives.begin(); i != m_drives.end(); ++i )
            {
                bool found = false;
                for ( re2robotDriver::DriverConfig::_drives_type::const_iterator j = msg.drives.begin(); j != msg.drives.end() && !found; ++j )
                {
                    found = ( i->first == j->name );
                }
                if ( !found )
                {
                    drivesRemoved.push_back( i->second );
                }
            }
            
            for ( std::vector< DriveProxyPtr >::const_iterator i = drivesRemoved.begin(); i != drivesRemoved.end(); ++i )
            {
                m_drives.erase( (*i)->config().name );
            }
            
            for ( re2robotDriver::DriverConfig::_drives_type::const_iterator i = msg.drives.begin(); i != msg.drives.end(); ++i )
            {
                if ( m_drives.find( i->name ) == m_drives.end() )
                {
                    DriveProxyPtr drive( new DriveProxy( m_node, *i ) );
                    m_drives[ i->name ] = drive;
                    drivesAdded.push_back( drive );
                }
            }
            
            if ( m_changeCB )
            {
                m_changeCB( drivesAdded, drivesRemoved );
            }
        }
        
    }
}
