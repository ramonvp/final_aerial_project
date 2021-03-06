#ifndef MOVE_UAV_H
#define MOVE_UAV_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <final_aerial_project/MoveUAVAction.h>
#include <final_aerial_project/path_planner.h>

namespace final_aerial_project
{

class MoveUAV
{
public:
    MoveUAV(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

protected:
    void executeMoveUAV(const MoveUAVGoalConstPtr &goal);

    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg );

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg );

    void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    bool targetReached(const geometry_msgs::Point & goal) const;

    geometry_msgs::Point lastWaypoint(const trajectory_msgs::MultiDOFJointTrajectory & plan) const;

    bool isQuaternionValid(const geometry_msgs::Quaternion& q);

    void makePlanCallback(const geometry_msgs::Point::ConstPtr & msg);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    actionlib::SimpleActionServer<MoveUAVAction> actionServer_;
    MoveUAVFeedback moveFeedback_;
    MoveUAVResult moveResult_;

    PathPlanner planner_;

    ros::Subscriber odom_sub_;
    ros::Publisher traj_pub_;
    ros::Subscriber rviz_goal_sub_;
    ros::Publisher traj_goal_pub_;

    geometry_msgs::PoseWithCovarianceStamped currentPose_;

    double target_tolerance_;

    ros::Subscriber infoSub_;
};


} // end namespace

#endif
