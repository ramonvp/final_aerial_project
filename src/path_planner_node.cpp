#include "final_aerial_project/path_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    FLAGS_alsologtostderr = true;

    ROS_INFO("Running path planner node...");
    final_aerial_project::PathPlanner planner_node(nh, nh_private);

    ros::Rate rate(1.0);

    while(ros::ok())
    {
        ros::spinOnce();

        planner_node.loop();

        rate.sleep();

    }

    //ros::spin();
    
    return 0;
}
