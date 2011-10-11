#include <re2robotModel/ModelProxy.h>
#include <re2robotModel/AddTransmission.h>
#include <re2robotModel/RemoveTransmission.h>
#include <re2robotModel/ControlArbiterCmd.h>

namespace re2
{
    namespace robot
    {
        
        ModelProxy::ModelProxy( const ros::NodeHandle& node, const ChangeCB& changeCB ) :
            m_node( node ),
            m_driver( m_node ),
            m_changeCB( changeCB ),
            m_configSub( m_node.subscribe( "/re2/robot/model/config", 1, &ModelProxy::config, this ) ),
            m_addTransmissionClient( m_node.serviceClient< re2robotModel::AddTransmission >( "/re2/robot/model/add_transmission" ) ),
            m_removeTransmissionClient( m_node.serviceClient< re2robotModel::RemoveTransmission >( "/re2/robot/model/remove_transmission" ) ),
            m_controlArbiterStateSub( m_node.subscribe( "/re2/robot/model/arbiters/control/state", 1, &ModelProxy::controlArbiterState, this ) ),
            m_controlArbiterCmdPub( m_node.advertise< re2robotModel::ControlArbiterCmd >( "/re2/robot/model/arbiters/control/cmd", 1 ) )
        {
            ROS_ASSERT_MSG( m_model.initParam( "/robot_description" ), "failed to init urdf model" );
        }
        
        ModelProxy::~ModelProxy()
        {
        }
        
        ros::NodeHandle& ModelProxy::node()
        {
            return m_node;
        }
        
        DriverProxy& ModelProxy::driver()
        {
            return m_driver;
        }
        
        urdf::Model& ModelProxy::model()
        {
            return m_model;
        }
        
        TransmissionProxyPtr ModelProxy::addTransmission( const re2robotModel::TransmissionConfig& config )
        {
            re2robotModel::AddTransmission srv;
            srv.request.config = config;
            if ( m_addTransmissionClient.call( srv ) && srv.response.success )
            {
                TransmissionProxyPtr transmission( new TransmissionProxy( this, config ) );
                m_transmissions[ transmission->config().joint ] = transmission;
                if ( m_changeCB )
                {
                    std::vector< TransmissionProxyPtr > transmissionsAdded, transmissionsRemoved;
                    transmissionsAdded.push_back( transmission );
                    m_changeCB( transmissionsAdded, transmissionsRemoved );
                }
                return transmission;
            }
            else
            {
                return TransmissionProxyPtr();
            }
        }
        
        bool ModelProxy::removeTransmission( const std::string& joint )
        {
            re2robotModel::RemoveTransmission srv;
            srv.request.joint = joint;
            if ( m_removeTransmissionClient.call( srv ) && srv.response.success )
            {
                std::map< std::string, TransmissionProxyPtr >::iterator i = m_transmissions.find( joint );
                if ( i != m_transmissions.end() )
                {
                    if ( m_changeCB )
                    {
                        std::vector< TransmissionProxyPtr > transmissionsAdded, transmissionsRemoved;
                        transmissionsRemoved.push_back( i->second );
                        m_changeCB( transmissionsAdded, transmissionsRemoved );
                    }
                    m_transmissions.erase( i );
                }
                return true;
            }
            else
            {
                return false;
            }
        }
        
        TransmissionProxyPtr ModelProxy::getTransmission( const std::string& joint ) const
        {
            std::map< std::string, TransmissionProxyPtr >::const_iterator i = m_transmissions.find( joint );
            if ( i == m_transmissions.end() )
            {
                return TransmissionProxyPtr();
            }
            else
            {
                return i->second;
            }
        }
        
        ModelProxy::TransmissionIterator ModelProxy::transmissionsBegin() const
        {
            return m_transmissions.begin();
        }
        
        ModelProxy::TransmissionIterator ModelProxy::transmissionsEnd() const
        {
            return m_transmissions.end();
        }
        
        void ModelProxy::setChangeCB( const ChangeCB& changeCB )
        {
            m_changeCB = changeCB;
        }
        
        void ModelProxy::config( const re2robotModel::ModelConfig& msg )
        {
            std::vector< TransmissionProxyPtr > transmissionsAdded, transmissionsRemoved;
            for ( std::map< std::string, TransmissionProxyPtr >::const_iterator i = m_transmissions.begin(); i != m_transmissions.end(); ++i )
            {
                bool found = false;
                for ( re2robotModel::ModelConfig::_transmissions_type::const_iterator j = msg.transmissions.begin(); j != msg.transmissions.end(); ++j )
                {
                    found = ( i->first == j->joint );
                }
                if ( !found )
                {
                    transmissionsRemoved.push_back( i->second );
                }
            }
            for ( std::vector< TransmissionProxyPtr >::const_iterator i = transmissionsRemoved.begin(); i != transmissionsRemoved.end(); ++i )
            {
                m_transmissions.erase( (*i)->config().joint );
            }
            
            for ( re2robotModel::ModelConfig::_transmissions_type::const_iterator i = msg.transmissions.begin(); i != msg.transmissions.end(); ++i )
            {
                if ( m_transmissions.find( i->joint ) == m_transmissions.end() )
                {
                    TransmissionProxyPtr transmission( new TransmissionProxy( this, *i ) );
                    transmissionsAdded.push_back( transmission );
                    m_transmissions[ transmission->config().joint ] = transmission;
                }
            }
            
            if ( m_changeCB )
            {
                m_changeCB( transmissionsAdded, transmissionsRemoved );
            }
        }
        
        void ModelProxy::controlArbiterState( const re2robotModel::ControlArbiterState& msg )
        {
            m_controlArbiterState = msg;
        }
        
        const re2robotModel::ControlArbiterState& ModelProxy::controlArbiterState() const
        {
            return m_controlArbiterState;
        }
        
        void ModelProxy::setActiveController( const std::string& controller )
        {
            re2robotModel::ControlArbiterCmd msg;
            msg.mode = re2robotModel::ControlArbiterCmd::MODE_SET_ACTIVE_CONTROLLER;
            msg.controller = controller;
            m_controlArbiterCmdPub.publish( msg );
        }
        
        void ModelProxy::addController( const std::string& controller )
        {
            re2robotModel::ControlArbiterCmd msg;
            msg.mode = re2robotModel::ControlArbiterCmd::MODE_ADD_CONTROLLER;
            msg.controller = controller;
            m_controlArbiterCmdPub.publish( msg );
        }
        
        void ModelProxy::removeController( const std::string& controller )
        {
            re2robotModel::ControlArbiterCmd msg;
            msg.mode = re2robotModel::ControlArbiterCmd::MODE_REMOVE_CONTROLLER;
            msg.controller = controller;
            m_controlArbiterCmdPub.publish( msg );
        }
        
    }
}
