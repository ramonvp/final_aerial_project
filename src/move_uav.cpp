#include "final_aerial_project/move_uav.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace final_aerial_project
{

MoveUAV::MoveUAV(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    actionServer_(nh_, "move_uav", boost::bind(&MoveUAV::executeMoveUAV, this, _1), false),
    planner_(nh, nh_private)
{
    ROS_INFO("MoveUAV loading");

    actionServer_.start();

    std::string pose_topic = "/firefly/ground_truth/pose_with_covariance";
    pose_sub_ =  nh_.subscribe(pose_topic, 1, &MoveUAV::poseCallback, this );
    rviz_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &MoveUAV::rvizGoalCallback, this);
    traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);

    traj_goal_pub_ = nh_.advertise<final_aerial_project::MoveUAVActionGoal>("move_uav/goal", 1);

    target_tolerance_ = 0.5;
}

bool MoveUAV::targetReached(const geometry_msgs::Point & goal) const
{
    return planner_.euclideanDistance(goal, currentPose_.pose.pose.position) <= target_tolerance_;
}

geometry_msgs::Point MoveUAV::lastWaypoint(const trajectory_msgs::MultiDOFJointTrajectory & plan) const
{
   geometry_msgs::Point p;

   const trajectory_msgs::MultiDOFJointTrajectoryPoint & last = *plan.points.rbegin();

   p.x = last.transforms[0].translation.x;
   p.y = last.transforms[0].translation.y;
   p.z = last.transforms[0].translation.z;

   return p;
}


void MoveUAV::executeMoveUAV(const MoveUAVGoalConstPtr &goal)
{
    ROS_INFO("Move UAV Goal received");

    double desired_z = goal->target_pose.pose.position.z;

    while( ! targetReached(goal->target_pose.pose.position) )
    {
        trajectory_msgs::MultiDOFJointTrajectory flight_plan;
        geometry_msgs::PoseStamped start_pose;
        start_pose.header = currentPose_.header;
        start_pose.pose = currentPose_.pose.pose;
        start_pose.pose.position.z = desired_z;
        bool plan_ok = planner_.makePlan(start_pose, goal->target_pose, flight_plan);
        if(plan_ok)
        {
            traj_pub_.publish(flight_plan);

            ros::Rate r(10);
            while( ! targetReached(lastWaypoint(flight_plan)) )
            {
                ros::spinOnce();
                r.sleep();
            }
            ROS_INFO("[MOVE_UAV] Partial last waypoint reached!");
        }
        else {
            ROS_ERROR("[MOVE_UAV] Path planner failed to produce a plan!");
            moveResult_.result = MoveUAVResult::RESULT_ERR_NO_PLAN;
            break;
        }

    }

    if(targetReached(goal->target_pose.pose.position))
    {
        ROS_INFO("[MOVE_UAV] Target reached!");
        moveResult_.result = MoveUAVResult::RESULT_SUCCESS;
        actionServer_.setSucceeded(moveResult_);
    }
    else {
        actionServer_.setAborted(moveResult_);
    }

    //actionServer_.setPreempted();
}

void MoveUAV::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg )
{
    ROS_INFO_ONCE("[MOVE_UAV] First pose msg received");
    currentPose_ = *msg;
}

void MoveUAV::rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ROS_INFO("[MOVE_UAV] Goal from RViz received");

    MoveUAVActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *msg;
    action_goal.goal.target_pose.pose.position.z = currentPose_.pose.pose.position.z;
    traj_goal_pub_.publish(action_goal);
}

}
