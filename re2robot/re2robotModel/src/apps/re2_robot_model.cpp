#include <re2robotModel/Model.h>

int main( int argc, char** argv )
{
    ros::init( argc, argv, "re2_robot_model" );
    ros::NodeHandle node;
    re2::robot::Model model( node );
    ros::spin();
    return 0;
}
