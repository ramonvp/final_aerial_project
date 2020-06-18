#include "aerial_project/path_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    FLAGS_alsologtostderr = true;

    ROS_INFO("Running path planner node...");
    voxblox::PathPlanner planner_node(nh, nh_private);

    ros::spin();
    
    return 0;
}