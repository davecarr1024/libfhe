#include <re2robotModel/ModelProxy.h>
#include <re2math/RE2Math.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace re2;
using namespace robot;

boost::shared_ptr< ModelProxy > model;

TEST( ModelTest, motion )
{
    ASSERT_TRUE( model );
    
    static const double POSITION_THRESHOLD = 0.01;
    static const double VELOCITY_THRESHOLD = 0.01;
    static const std::string CONTROLLER = "ModelTest";
    double startPosition;
    double endPosition;
    double maxVelocity;
    bool controllerFound;
    
    TransmissionProxyPtr testTransmission;
    for ( size_t i = 0; i < 20 && !testTransmission; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
        testTransmission = model->getTransmission( "mpto" );
    }
    ASSERT_TRUE( testTransmission );
    
    maxVelocity = testTransmission->drive()->config().maxVelocity;
    
    //assert we can't move without controller set
    startPosition = testTransmission->state().position;
    for ( size_t i = 0; i <  20 && testTransmission->state().position < startPosition + maxVelocity / 2.; ++i )
    {
        testTransmission->setEffort( 1, CONTROLLER );
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
    }
    ASSERT_LT( testTransmission->state().position, startPosition + maxVelocity / 2. );
    
    //add controller
    model->addController( CONTROLLER );
    controllerFound = false;
    for ( size_t i = 0; i < 20 && !controllerFound; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
        for ( size_t j = 0; j < model->controlArbiterState().controllers.size() && !controllerFound; ++j )
        {
            controllerFound = ( model->controlArbiterState().controllers[j] == CONTROLLER );
        }
    }
    ASSERT_TRUE( controllerFound );
    
    //set active controller
    model->setActiveController( CONTROLLER );
    for ( size_t i = 0; i < 20 && model->controlArbiterState().activeController != CONTROLLER; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
    }
    ASSERT_EQ( CONTROLLER, model->controlArbiterState().activeController );
    
    //move with effort
    for ( size_t i = 0; i <  20 && testTransmission->state().position < startPosition + maxVelocity / 2.; ++i )
    {
        testTransmission->setEffort( 1, CONTROLLER );
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
    }
    ASSERT_GT( testTransmission->state().position, startPosition + maxVelocity / 2. );
    
    //stop
    for ( size_t i = 0; i < 20 && Math::abs( testTransmission->state().velocity ) > VELOCITY_THRESHOLD; ++i )
    {
        testTransmission->setVelocity( 0, CONTROLLER );
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
    }
    ASSERT_NEAR( testTransmission->state().velocity, 0, VELOCITY_THRESHOLD );
    
    //move with velocity
    startPosition = testTransmission->state().position;
    for ( size_t i = 0; i < 20 && testTransmission->state().position > startPosition - maxVelocity / 2.; ++i )
    {
        testTransmission->setVelocity( -maxVelocity, CONTROLLER );
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
    }
    ASSERT_LT( testTransmission->state().position, startPosition - maxVelocity / 2. );
    
    //stop
    for ( size_t i = 0; i < 20 && Math::abs( testTransmission->state().velocity ) > VELOCITY_THRESHOLD; ++i )
    {
        testTransmission->setVelocity( 0, CONTROLLER );
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
    }
    ASSERT_NEAR( testTransmission->state().velocity, 0, VELOCITY_THRESHOLD );
    
    //move to position
    endPosition = testTransmission->state().position + 1;
    for ( size_t i = 0; i < 20 && Math::abs( testTransmission->state().position - endPosition ) > POSITION_THRESHOLD; ++i )
    {
        testTransmission->setPosition( endPosition, CONTROLLER );
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
    }
    ASSERT_NEAR( testTransmission->state().position, endPosition, POSITION_THRESHOLD );
    
    //remove controller
    model->removeController( CONTROLLER );
    controllerFound = true;
    for ( size_t i = 0; i < 20 && controllerFound; ++i )
    {
        ros::Duration( 0.1 ).sleep();
        ros::spinOnce();
        controllerFound = false;
        for ( size_t j = 0; j < model->controlArbiterState().controllers.size() && !controllerFound; ++j )
        {
            controllerFound = ( model->controlArbiterState().controllers[j] == CONTROLLER );
        }
    }
    ASSERT_FALSE( controllerFound );
    
    //remove transmission
    ASSERT_TRUE( model->removeTransmission( "mpto" ) );
    ASSERT_FALSE( model->getTransmission( "mpto" ) );
}

int main( int argc, char** argv )
{
    ::testing::InitGoogleTest( &argc, argv );
    ros::init( argc, argv, "ModelTest" );
    ros::NodeHandle node;
    model.reset( new ModelProxy( node ) );
    return RUN_ALL_TESTS();
}
