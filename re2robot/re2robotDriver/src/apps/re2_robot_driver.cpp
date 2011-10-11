#include <re2robotDriver/Driver.h>

using namespace re2::robot;

int main( int argc, char** argv )
{
    ros::init( argc, argv, "re2_robot_driver" );
    ros::NodeHandle node;
    Driver driver( node );
    ros::spin();
    return 0;
}
