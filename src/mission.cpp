#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "final_aerial_project/ProductInfo.h"
#include "final_aerial_project/ProductFeedback.h"
#include <std_msgs/Empty.h>

class StateMachine
{
public:

  enum State {
      INITIALIZING,
      WAITING_ORDER,
      FLYING_TO_TARGET,
      FLYING_TO_RECHARGE,
      SEARCHING_TARGET,
      DONE
  };

  StateMachine() : state_{INITIALIZING}
  {

  }

  void signalTakeOff()
  {

  }

  State state_;
};

class MissionPlanner
{
public:
    MissionPlanner(const ros::NodeHandle & nh, const ros::NodeHandle & nh_private)
      : nh_(nh),
        nh_params_(nh_private),
        detectionThreshold_{1.0},
        lowBatteryThreshold_{40},
        takeOffSent_{false},
        takeOffHeight_{1.0}
    {
          dronePoseSub_ = nh_.subscribe("odom", 1, &MissionPlanner::odomCallback, this);
          batterySub_ = nh_.subscribe("/firefly/battery_timer", 1, &MissionPlanner::batteryCallback, this);
          productFeedbackPub_ = nh_.advertise<final_aerial_project::ProductFeedback>("/firefly/product_feedback", 1);
          productInfoSub_ = nh_.subscribe("/parcel_dispatcher/next_product", 1, &MissionPlanner::dispatcherCallback, this);
          //kalmanReadySub_ = nh_.subscribe("filter_ready", 1, &MissionPlanner::kalmanReadyCallback, this);
          poseCommandPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);


          ROS_INFO("[MissionPlanner] Ready");
    }

    void kalmanReadyCallback(const std_msgs::Empty::ConstPtr & msg)
    {

    }

    void batteryCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        remainingBattery_ = msg->data;
        if( remainingBattery_ <= lowBatteryThreshold_ )
        {
            // TODO: go back to charging pad
            ROS_WARN("[MissionPlanner] Low battery");
        }
    }

    void dispatcherCallback(const final_aerial_project::ProductInfo::ConstPtr &product_msg)
    {
        //msg->item_location : std::string
        //msg->marker_id : int64
        //msg->approximate_pose : geometry_msgs/PoseWithCovariance
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        ROS_INFO_ONCE("[MissionPlanner] First odom message received");
        currentOdom_ = *msg;
        if(!takeOffSent_)
        {
            ROS_INFO("[MissionPlanner] Take Off!");
            geometry_msgs::PoseStamped takeOffMsg;
            takeOffMsg.header = currentOdom_.header;
            takeOffMsg.pose = currentOdom_.pose.pose;
            takeOffMsg.pose.position.z = takeOffHeight_;
            poseCommandPub_.publish(takeOffMsg);
            takeOffSent_ = true;
        }
    }

    void loop()
    {
        // run state machine
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_params_;

    ros::Subscriber dronePoseSub_;
    ros::Subscriber batterySub_;
    //ros::Subscriber kalmanReadySub_;
    ros::Publisher productFeedbackPub_;
    ros::Publisher poseCommandPub_;
    ros::Subscriber productInfoSub_;

    int32_t remainingBattery_;

    // detection_threshold from parameter server
    float detectionThreshold_;

    int32_t lowBatteryThreshold_;

    nav_msgs::Odometry currentOdom_;

    StateMachine sm_;

    bool takeOffSent_;
    double takeOffHeight_;
};




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mission_planner");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    MissionPlanner mission_planner(nh, nh_private);

    ros::Rate rate(10);
    while(ros::ok())
    {
        rate.sleep();
        mission_planner.loop();
        ros::spinOnce();
    }

    return 0;
}
