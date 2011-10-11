#include <re2robotModel/Arbiter.h>
#include <re2robotModel/Model.h>

namespace re2
{
    namespace robot
    {
        
        Arbiter::Arbiter()
        {
        }
        
        Arbiter::~Arbiter()
        {
        }
        
        bool Arbiter::init( Model* model, const re2robotModel::ArbiterConfig& config )
        {
            ROS_ASSERT( m_model = model );
            m_config = config;
            
            m_configPub = m_model->node().advertise< re2robotModel::ArbiterConfig >( "/re2/robot/model/arbiters/" + m_config.type + "/config", 1, true );
            m_configPub.publish( m_config );
            
            m_timer = m_model->node().createTimer( ros::Duration( config.updateIntervalS ), &Arbiter::update, this );
            
            return init();
        }
        
        Model* Arbiter::model() const
        {
            return m_model;
        }
        
        const re2robotModel::ArbiterConfig& Arbiter::config() const
        {
            return m_config;
        }
        
        bool Arbiter::init()
        {
            return true;
        }
        
        void Arbiter::update()
        {
        }
        
        void Arbiter::shutdown()
        {
        }
        
        void Arbiter::update( const ros::TimerEvent& e )
        {
            update();
        }
        
    }
}
