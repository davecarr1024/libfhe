#include <re2robotModel/TransmissionProxy.h>
#include <re2robotModel/ModelProxy.h>

namespace re2
{
    namespace robot
    {
        
        TransmissionProxy::TransmissionProxy( ModelProxy* model, const re2robotModel::TransmissionConfig& config ) :
            m_model( model ),
            m_config( config ),
            m_cmdPub( m_model->node().advertise< re2robotModel::JointCmd >( "/re2/robot/model/joints/" + m_config.joint + "/cmd", 1 ) ),
            m_stateSub( m_model->node().subscribe( "/re2/robot/model/joints/" + m_config.joint + "/state", 1, &TransmissionProxy::state, this ) )
        {
        }
        
        TransmissionProxy::~TransmissionProxy()
        {
        }
        
        ModelProxy* TransmissionProxy::model() const
        {
            return m_model;
        }
        
        const re2robotModel::TransmissionConfig& TransmissionProxy::config() const
        {
            return m_config;
        }
        
        const re2robotModel::JointState& TransmissionProxy::state() const
        {
            return m_state;
        }
        
        DriveProxyPtr TransmissionProxy::drive() const
        {
            return m_model->driver().getDrive( config().drive );
        }
        
        boost::shared_ptr< const urdf::Joint > TransmissionProxy::joint() const
        {
            return m_model->model().getJoint( config().joint );
        }
        
        void TransmissionProxy::setEffort( double effort, const std::string& controller )
        {
            re2robotModel::JointCmd msg;
            msg.mode = re2robotModel::JointCmd::MODE_EFFORT;
            msg.cmd = effort;
            msg.controller = controller;
            m_cmdPub.publish( msg );
        }
        
        void TransmissionProxy::setVelocity( double velocity, const std::string& controller )
        {
            re2robotModel::JointCmd msg;
            msg.mode = re2robotModel::JointCmd::MODE_VELOCITY;
            msg.cmd = velocity;
            msg.controller = controller;
            m_cmdPub.publish( msg );
        }
        
        void TransmissionProxy::setPosition( double position, const std::string& controller )
        {
            re2robotModel::JointCmd msg;
            msg.mode = re2robotModel::JointCmd::MODE_POSITION;
            msg.cmd = position;
            msg.controller = controller;
            m_cmdPub.publish( msg );
        }
        
        void TransmissionProxy::cmd( const re2robotModel::JointCmd& msg )
        {
            m_cmdPub.publish( msg );
        }
        
        void TransmissionProxy::state( const re2robotModel::JointState& msg )
        {
            m_state = msg;
        }
        
    }
}
