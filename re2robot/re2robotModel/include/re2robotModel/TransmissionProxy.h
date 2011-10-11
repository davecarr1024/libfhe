#ifndef RE2_ROBOT_MODEL_TRANSMISSION_PROXY_H
#define RE2_ROBOT_MODEL_TRANSMISSION_PROXY_H

#include <re2robotModel/TransmissionConfig.h>
#include <re2robotModel/JointState.h>
#include <re2robotModel/JointCmd.h>
#include <re2robotDriver/DriveProxy.h>
#include <urdf/joint.h>

namespace re2
{
    namespace robot
    {
        
        class ModelProxy;
        
        class TransmissionProxy : public boost::noncopyable
        {
            friend class ModelProxy;
            
            public:
                virtual ~TransmissionProxy();
                
                ModelProxy* model() const;
                const re2robotModel::TransmissionConfig& config() const;
                const re2robotModel::JointState& state() const;
                DriveProxyPtr drive() const;
                boost::shared_ptr< const urdf::Joint > joint() const;
                
                void setEffort( double effort, const std::string& controller );
                void setVelocity( double velocity, const std::string& controller );
                void setPosition( double position, const std::string& controller );
                void cmd( const re2robotModel::JointCmd& msg );
                
            private:
                TransmissionProxy( ModelProxy* model, const re2robotModel::TransmissionConfig& config );
                
                ModelProxy* m_model;
                const re2robotModel::TransmissionConfig m_config;
                re2robotModel::JointState m_state;
                
                ros::Publisher m_cmdPub;
                
                ros::Subscriber m_stateSub;
                void state( const re2robotModel::JointState& state );
        };
        
        typedef boost::shared_ptr< TransmissionProxy > TransmissionProxyPtr;
        
    }
}

#endif
