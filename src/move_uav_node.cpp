#include "final_aerial_project/move_uav.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_uav_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    ROS_INFO("Running move uav node...");
    final_aerial_project::MoveUAV move_uav_node(nh, nh_private);

#if 1
    ros::spin();
#else
    ros::Rate rate(1.0);

    while(ros::ok())
    {
        ros::spinOnce();

        planner_node.loop();

        rate.sleep();

    }
#endif


    return 0;
}

