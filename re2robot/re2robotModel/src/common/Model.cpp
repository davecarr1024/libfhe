#include <re2robotModel/Model.h>
#include <re2robotModel/ModelConfig.h>
#include <re2robotUtil/Config.h>

namespace re2
{
    namespace robot
    {
        
        #define DEFAULT_TRANSMISSION_INTERVAL 0.01
        
        Model::Model( const ros::NodeHandle& node, const std::string& robotDescPath ) :
            m_node( node ),
            m_driver( m_node ),
            m_transmissionLoader( "re2robotModel", "re2::robot::Transmission" ),
            m_arbiterLoader( "re2robotModel", "re2::robot::Arbiter" ),
            m_addTransmissionSrv( m_node.advertiseService( "/re2/robot/model/add_transmission", &Model::addTransmission, this ) ),
            m_removeTransmissionSrv( m_node.advertiseService( "/re2/robot/model/remove_transmission", &Model::removeTransmission, this ) ),
            m_addArbiterSrv( m_node.advertiseService( "/re2/robot/model/add_arbiter", &Model::addArbiter, this ) ),
            m_removeArbiterSrv( m_node.advertiseService( "/re2/robot/model/remove_arbiter", &Model::removeArbiter, this ) ),
            m_configPub( m_node.advertise< re2robotModel::ModelConfig >( "/re2/robot/model/config", 1, true ) )
        {
            ROS_ASSERT_MSG( m_model.initParam( "/robot_description" ), "unable to init urdf model" );
            
            if ( !robotDescPath.empty() )
            {
                Config config( robotDescPath ),
                    robotConfig( config[ "robot" ] ),
                    transmissionConfigs( robotConfig[ "transmission" ] ),
                    driveConfigs( robotConfig[ "drive" ] ),
                    arbiterConfigs( robotConfig[ "arbiter" ] );
                    
                for ( size_t i = 0; i < arbiterConfigs.size(); ++i )
                {
                    re2robotModel::ArbiterConfig config;
                    ROS_ASSERT_MSG( arbiterConfigs[i][ "type" ].tryTo< std::string >( config.type ),
                                    "unable to load type for arbiter" );
                    config.initialController = arbiterConfigs[i][ "initialController" ].to< std::string >( "" );
                    ROS_ASSERT_MSG( addArbiter( config ), "unable to add arbiter %s", config.type.c_str() );
                }
                    
                std::set< std::string > driveNames;
                for ( size_t i = 0; i < driveConfigs.size(); ++i )
                {
                    std::string driveName;
                    if ( driveConfigs[ i ][ "name" ].tryTo< std::string >( driveName ) )
                    {
                        driveNames.insert( driveName );
                    }
                }
                    
                for ( size_t i = 0; i < transmissionConfigs.size(); ++i )
                {
                    Config transmissionConfig( transmissionConfigs[ i ] );
                    re2robotModel::TransmissionConfig configOut;
                    
                    ROS_ASSERT_MSG( transmissionConfig[ "type" ].tryTo< std::string >( configOut.type ),
                                    "unable to load type for transmission" );
                                    
                    ROS_ASSERT_MSG( transmissionConfig[ "drive" ].tryTo< std::string >( configOut.drive ),
                                    "unable to load drive for transmission" );
                    ROS_ASSERT_MSG( driveNames.find( configOut.drive ) != driveNames.end(),
                                    "unknown transmission drive %s", configOut.drive.c_str() );
                                    
                    ROS_ASSERT_MSG( transmissionConfig[ "joint" ].tryTo< std::string >( configOut.joint ),
                                    "unable to load joint for transmission" );
                    ROS_ASSERT_MSG( m_model.getJoint( configOut.joint ),
                                    "unknown transmission joint %s", configOut.joint.c_str() );
                                    
                    configOut.timerIntervalS = transmissionConfig[ "timerIntervalS" ].to< double >( DEFAULT_TRANSMISSION_INTERVAL );
                    ROS_ASSERT_MSG( addTransmission( configOut ), 
                                    "unable to add transmission for joint %s", configOut.joint.c_str() );
                }
                
                for ( std::map< std::string, boost::shared_ptr< urdf::Joint > >::const_iterator i = m_model.joints_.begin(); i != m_model.joints_.end(); ++i )
                {
                    if ( m_transmissions.find( i->first ) == m_transmissions.end() && driveNames.find( i->first ) != driveNames.end() )
                    {
                        ROS_INFO( "adding default transmission for joint %s", i->first.c_str() );
                        re2robotModel::TransmissionConfig config;
                        config.type = "re2robotModel/SimpleTransmission";
                        config.drive = i->first;
                        config.joint = i->first;
                        config.timerIntervalS = DEFAULT_TRANSMISSION_INTERVAL;
                        ROS_ASSERT_MSG( addTransmission( config ), 
                                        "unable to add transmission for joint %s", config.joint.c_str() );
                    }
                }
                
            }
            
            sendConfig();
        }
        
        Model::~Model()
        {
        }
        
        ros::NodeHandle& Model::node()
        {
            return m_node;
        }
        
        DriverProxy& Model::driver()
        {
            return m_driver;
        }
        
        urdf::Model& Model::model()
        {
            return m_model;
        }
        
        TransmissionPtr Model::addTransmission( const re2robotModel::TransmissionConfig& config )
        {
            if ( m_transmissions.find( config.joint ) != m_transmissions.end() )
            {
                ROS_ERROR( "unable to add duplicate transmission for joint %s", config.joint.c_str() );
                return TransmissionPtr();
            }
            else
            {
                TransmissionPtr transmission;
                try
                {
                    transmission.reset( m_transmissionLoader.createClassInstance( config.type ) );
                }
                catch ( pluginlib::PluginlibException& e )
                {
                    ROS_ERROR( "unable to load transmission of type %s: %s", config.type.c_str(), e.what() );
                    return TransmissionPtr();
                }
                if ( !transmission->init( this, config ) )
                {
                    ROS_ERROR( "unable to init transmission for joint %s", config.joint.c_str() );
                    return TransmissionPtr();
                }
                else
                {
                    m_transmissions[ transmission->config().joint ] = transmission;
                    sendConfig();
                    return transmission;
                }
            }
        }
        
        bool Model::removeTransmission( const std::string& joint )
        {
            std::map< std::string, TransmissionPtr >::iterator i = m_transmissions.find( joint );
            if ( i == m_transmissions.end() )
            {
                ROS_ERROR( "unable to remove unknown transmission for joint %s", joint.c_str() );
                return false;
            }
            else
            {
                i->second->shutdown();
                m_transmissions.erase( i );
                sendConfig();
                return true;
            }
        }
        
        TransmissionPtr Model::getTransmission( const std::string& joint ) const
        {
            std::map< std::string, TransmissionPtr >::const_iterator i = m_transmissions.find( joint );
            if ( i == m_transmissions.end() )
            {
                return TransmissionPtr();
            }
            else
            {
                return i->second;
            }
        }
        
        Model::TransmissionIterator Model::transmissionsBegin() const
        {
            return m_transmissions.begin();
        }
        
        Model::TransmissionIterator Model::transmissionsEnd() const
        {
            return m_transmissions.end();
        }
        
        bool Model::addTransmission( re2robotModel::AddTransmission::Request& req,
                                     re2robotModel::AddTransmission::Response& res )
        {
            res.success = (bool)addTransmission( req.config );
            return true;
        }
        
        bool Model::removeTransmission( re2robotModel::RemoveTransmission::Request& req,
                                        re2robotModel::RemoveTransmission::Response& res )
        {
            res.success = removeTransmission( req.joint );
            return true;
        }
        
        void Model::sendConfig()
        {
            re2robotModel::ModelConfig msg;
            for ( std::map< std::string, TransmissionPtr >::const_iterator i = m_transmissions.begin(); i != m_transmissions.end(); ++i )
            {
                msg.transmissions.push_back( i->second->config() );
            }
            for ( std::map< std::string, ArbiterPtr >::const_iterator i = m_arbiters.begin(); i != m_arbiters.end(); ++i )
            {
                msg.arbiters.push_back( i->second->config() );
            }
            m_configPub.publish( msg );
        }
        
        ArbiterPtr Model::addArbiter( const re2robotModel::ArbiterConfig& config )
        {
            if ( m_arbiters.find( config.type ) != m_arbiters.end() )
            {
                ROS_ERROR( "unable to add duplicate arbiter %s", config.type.c_str() );
                return ArbiterPtr();
            }
            else
            {
                ArbiterPtr arbiter;
                try
                {
                    arbiter.reset( m_arbiterLoader.createClassInstance( config.type ) );
                }
                catch ( pluginlib::PluginlibException& e )
                {
                    ROS_ERROR( "unable to load arbiter of type %s: %s", config.type.c_str(), e.what() );
                    return ArbiterPtr();
                }
                if ( !arbiter->init( this, config ) )
                {
                    ROS_ERROR( "failed to init arbiter %s", arbiter->config().type.c_str() );
                    return ArbiterPtr();
                }
                else
                {
                    m_arbiters[ arbiter->config().type ] = arbiter;
                    sendConfig();
                    return arbiter;
                }
            }
        }
        
        bool Model::removeArbiter( const std::string& type )
        {
            std::map< std::string, ArbiterPtr >::iterator i = m_arbiters.find( type );
            if ( i == m_arbiters.end() )
            {
                return false;
            }
            else
            {
                i->second->shutdown();
                m_arbiters.erase( i );
                sendConfig();
                return true;
            }
        }
        
        bool Model::addArbiter( re2robotModel::AddArbiter::Request& req,
                                re2robotModel::AddArbiter::Response& res )
        {
            res.success = (bool)addArbiter( req.config );
            return true;
        }
        
        bool Model::removeArbiter( re2robotModel::RemoveArbiter::Request& req,
                                   re2robotModel::RemoveArbiter::Response& res )
        {
            res.success = removeArbiter( req.type );
            return true;
        }
        
        bool Model::forward( const std::string& joint, const re2robotModel::JointCmd& cmd ) const
        {
            bool forward = true;
            for ( std::map< std::string, ArbiterPtr >::const_iterator i = m_arbiters.begin(); i != m_arbiters.end() && forward; ++i )
            {
                forward = i->second->forward( joint, cmd );
            }
            return forward;
        }
        
    }
}
