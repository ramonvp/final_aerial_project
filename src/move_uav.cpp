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

    bool enable_rviz_goal;
    nh_private_.param("enable_rviz_goal", enable_rviz_goal, true);
    if(enable_rviz_goal)
        rviz_goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &MoveUAV::rvizGoalCallback, this);

    std::string pose_topic = "/firefly/ground_truth/pose_with_covariance";
    //pose_sub_ =  nh_.subscribe(pose_topic, 1, &MoveUAV::poseCallback, this );
    odom_sub_ =  nh_.subscribe("odom", 1, &MoveUAV::odomCallback, this );
    traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);

    traj_goal_pub_ = nh_.advertise<final_aerial_project::MoveUAVActionGoal>("move_uav/goal", 1);

    nh_private_.param("target_tolerance", target_tolerance_, 0.2);

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

bool MoveUAV::isQuaternionValid(const geometry_msgs::Quaternion& q){
  //first we need to check if the quaternion has nan's or infs
  if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
    ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
    return false;
  }

  tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

  //next, we need to check if the length of the quaternion is close to zero
  if(tf_q.length2() < 1e-6){
    ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
    return false;
  }

  //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
  tf_q.normalize();

  tf::Vector3 up(0, 0, 1);

  double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

  if(fabs(dot - 1) > 1e-3){
    ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
    return false;
  }

  return true;
}

void MoveUAV::executeMoveUAV(const MoveUAVGoalConstPtr & move_uav_goal)
{
    ROS_INFO("Move UAV Goal received");

    if(!isQuaternionValid(move_uav_goal->target_pose.pose.orientation))
    {
        moveResult_.result = MoveUAVResult::RESULT_ERR_NO_PLAN;
        actionServer_.setAborted(moveResult_, "Aborting on goal because it was sent with an invalid quaternion");
        return;
    }

    geometry_msgs::PoseStamped goal = move_uav_goal->target_pose;

    double desired_z = goal.pose.position.z;

    planner_.publishGoalPointMarker(goal);

    while( ! targetReached(goal.pose.position) )
    {
        trajectory_msgs::MultiDOFJointTrajectory flight_plan;
        geometry_msgs::PoseStamped start_pose;
        start_pose.header = currentPose_.header;
        start_pose.pose = currentPose_.pose.pose;
        start_pose.pose.position.z = desired_z;
        bool plan_ok = planner_.makePlan(start_pose, goal, flight_plan);
        if(plan_ok)
        {
            traj_pub_.publish(flight_plan);

            ros::Rate r(10);
            int numTargetsOk = 0;
            //while( ! targetReached(lastWaypoint(flight_plan)) )
            // wait 1 second until target reached is stable
            while(numTargetsOk < 10)
            {
                ros::spinOnce();
                r.sleep();
                if(targetReached(lastWaypoint(flight_plan)))
                {
                    numTargetsOk++;
                }
                else
                {
                   numTargetsOk = 0;
                }

                if(actionServer_.isPreemptRequested())
                {
                    if(actionServer_.isNewGoalAvailable())
                    {
                        //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                        MoveUAVGoalConstPtr new_goal = actionServer_.acceptNewGoal();

                        if(!isQuaternionValid(new_goal->target_pose.pose.orientation))
                        {
                            actionServer_.setAborted(moveResult_, "Aborting on goal because it was sent with an invalid quaternion");
                            return;
                        }
                        else
                        {
                            goal = new_goal->target_pose;
                            break; // re-calculate flight plan
                        }
                    }
                    else
                    {
                        // request to preempt, exit then
                        actionServer_.setPreempted();
                        return;
                    }
                }

            }
            ROS_INFO("[MOVE_UAV] Partial last waypoint reached!");
        }
        else {
            ROS_ERROR("[MOVE_UAV] Path planner failed to produce a plan!");
            moveResult_.result = MoveUAVResult::RESULT_ERR_NO_PLAN;
            break;
        }

    }

    if(targetReached(goal.pose.position))
    {
        ROS_INFO("[MOVE_UAV] Target reached!");
        moveResult_.result = MoveUAVResult::RESULT_SUCCESS;
        actionServer_.setSucceeded(moveResult_);
    }
    else {
        actionServer_.setAborted(moveResult_);
    }
}

void MoveUAV::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg )
{
    ROS_INFO_ONCE("[MOVE_UAV] First pose msg received");
    currentPose_ = *msg;
}
void MoveUAV::odomCallback(const nav_msgs::Odometry::ConstPtr& msg )
{
    ROS_INFO_ONCE("[MOVE_UAV] First odom msg received");
    currentPose_.pose = msg->pose;
    currentPose_.header = msg->header;
}

void MoveUAV::rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ROS_INFO("[MOVE_UAV] Goal received: %g,%g,%g  %g,%g,%g,%",
             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
             msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    MoveUAVActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *msg;
    // RViz 2D goals don't contain Z information, in such case, just assume goal z is
    // the same as current pose z.
    if(msg->pose.position.z == 0.0)
        action_goal.goal.target_pose.pose.position.z = currentPose_.pose.pose.position.z;
    traj_goal_pub_.publish(action_goal);
}

}
