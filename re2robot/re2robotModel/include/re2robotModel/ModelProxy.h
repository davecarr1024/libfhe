#ifndef RE2_ROBOT_MODEL_MODEL_PROXY_H
#define RE2_ROBOT_MODEL_MODEL_PROXY_H

#include <re2robotModel/TransmissionProxy.h>
#include <re2robotModel/ModelConfig.h>
#include <re2robotModel/ControlArbiterState.h>
#include <re2robotDriver/DriverProxy.h>
#include <urdf/model.h>

namespace re2
{
    namespace robot
    {
        
        class ModelProxy : public boost::noncopyable
        {
            public:
                typedef boost::function< void( const std::vector< TransmissionProxyPtr >& transmissionsAdded,
                                               const std::vector< TransmissionProxyPtr >& transmissionsRemoved ) > ChangeCB;
                
                ModelProxy( const ros::NodeHandle& node, const ChangeCB& changeCB = ChangeCB() );
                virtual ~ModelProxy();
                
                ros::NodeHandle& node();
                DriverProxy& driver();
                urdf::Model& model();
                
                TransmissionProxyPtr addTransmission( const re2robotModel::TransmissionConfig& config );
                bool removeTransmission( const std::string& joint );
                
                TransmissionProxyPtr getTransmission( const std::string& joint ) const;
                
                typedef std::map< std::string, TransmissionProxyPtr >::const_iterator TransmissionIterator;
                TransmissionIterator transmissionsBegin() const;
                TransmissionIterator transmissionsEnd() const;
                
                void setChangeCB( const ChangeCB& changeCB );

                const re2robotModel::ControlArbiterState& controlArbiterState() const;
                
                void setActiveController( const std::string& controller );
                void addController( const std::string& controller );
                void removeController( const std::string& controller );
                
            private:
                ros::NodeHandle m_node;
                DriverProxy m_driver;
                urdf::Model m_model;
                ChangeCB m_changeCB;
                std::map< std::string, TransmissionProxyPtr > m_transmissions;
                re2robotModel::ControlArbiterState m_controlArbiterState;
                
                ros::Subscriber m_configSub;
                void config( const re2robotModel::ModelConfig& msg );
                
                ros::ServiceClient m_addTransmissionClient;
                
                ros::ServiceClient m_removeTransmissionClient;
                
                ros::Subscriber m_controlArbiterStateSub;
                void controlArbiterState( const re2robotModel::ControlArbiterState& msg );
                
                ros::Publisher m_controlArbiterCmdPub;
        };
        
    }
}

#endif
