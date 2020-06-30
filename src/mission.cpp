#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "final_aerial_project/ProductInfo.h"
#include "final_aerial_project/ProductFeedback.h"
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <fiducial_msgs/FiducialTransformArray.h>

class StateMachine
{
public:


  enum State {
      INITIALIZING,
      WAITING_ORDER,
      FLYING_TO_TARGET,
      FLYING_TO_RECHARGE,
      FLYING_TO_CHECKPAD,
      SEARCHING_TARGET,
      DONE
  };

  enum Events {
      TAKE_OFF,
      ORDER_RECEIVED,
      TARGET_REACHED,
      OBJECT_FOUND,
      CHARGING_REACHED
  };

  StateMachine() : state_{INITIALIZING}
  {

  }

  const char * getStateName(State state)
  {
      switch(state_)
      {
          case INITIALIZING: return "INITIALIZING";
          case WAITING_ORDER: return "WAITING_ORDER";
          case FLYING_TO_TARGET: return "FLYING_TO_TARGET";
          case FLYING_TO_RECHARGE: return "FLYING_TO_RECHARGE";
          case SEARCHING_TARGET: return "SEARCHING_TARGET";
          case DONE: return "DONE";

          return "UNKNOWN";
      }
  }

  void transitionTo(State new_state)
  {
      ROS_INFO("[StateMachine] Transition from %s to %s",
               getStateName(state_), getStateName(new_state));
      state_ = new_state;
  }

  void StateInitializing(Events event)
  {
      if( event == Events::TAKE_OFF )
      {
          transitionTo(WAITING_ORDER);
      }
  }

  void StateWaitingOrder(Events event)
  {
      if( event == Events::ORDER_RECEIVED )
      {
          transitionTo(FLYING_TO_TARGET);
      }
      /*
      else if( event == Events::TARGET_REACHED )
      {
          transitionTo(WAITING_ORDER);
      }
      */
  }

  void StateFlyingToTarget(Events event)
  {
      if( event == Events::TARGET_REACHED )
      {
          transitionTo(SEARCHING_TARGET);
      }
      else if( event == Events::CHARGING_REACHED )
      {
          transitionTo(DONE);
      }
  }

  void StateSearchingTarget(Events event)
  {
      if( event == Events::OBJECT_FOUND )
      {
          transitionTo(FLYING_TO_CHECKPAD);
      }
  }

  void StateFlyingToCheckpad(Events event)
  {
      if( event == Events::TARGET_REACHED )
      {
          transitionTo(WAITING_ORDER);
      }
  }

  void signalEvent(Events event)
  {
      switch(state_)
      {
          case INITIALIZING: StateInitializing(event); break;
          case WAITING_ORDER: StateWaitingOrder(event); break;
          case FLYING_TO_TARGET: StateFlyingToTarget(event); break;
      }
  }

  State state() const { return state_;}

protected:
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

          plannerPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

          trajectoryPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);


          frontalCameraArucoSub_ = nh_.subscribe("/frontal/fiducial_transforms", 1, &MissionPlanner::frontalArucoCallback, this);
          ventralCameraArucoSub_ = nh_.subscribe("/ventral/fiducial_transforms", 1, &MissionPlanner::ventralArucoCallback, this);
          // TODO: fill shelve trajectory :
          // 15, 3, 1.0
          // 15, 3, 3.7
          // 17, 3, 3.7
          // 17, 3, 1.0
          //shelveTrajectory_

          // TODO: fill huskyTrajectory_
          // huskyTrajectory_

          // TODO: Read the coordinates from parameter server
          checkPadPose_.header.frame_id = "world";
          checkPadPose_.pose.position.x = 21.82;
          checkPadPose_.pose.position.y = 3.13;
          checkPadPose_.pose.position.z = 0.055;
          checkPadPose_.pose.orientation.w = 1.0;

          ROS_INFO("[MissionPlanner] Ready");
    }

    void frontalArucoCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg)
    {
        if( msg->transforms.size() == 0) return;

        if( sm_.state() == StateMachine::SEARCHING_TARGET && nextProduct_.item_location == "SHELVE")
        {
            for(auto & t : msg->transforms)
            {
                arucoPositions_[t.fiducial_id] = arucoWorldPosition(t.transform.translation, msg->header.frame_id);
            }

            bool fiducial_found = arucoPositions_.count(nextProduct_.marker_id);
            if(fiducial_found)
            {
                // dirigirme al check pad
                plannerPub_.publish(checkPadPose_);
                sm_.signalEvent(StateMachine::Events::OBJECT_FOUND);
            }
        }
    }

    geometry_msgs::Point arucoWorldPosition(geometry_msgs::Vector3 vector, std::string frame_id)
    {
        geometry_msgs::Point p;

        // TODO: TF lookup of aruco pos
        p.x = vector.x;
        p.y = vector.y;
        p.z = vector.z;

        return p;
    }

    void ventralArucoCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg)
    {
        if( msg->transforms.size() == 0) return;

        if( sm_.state() == StateMachine::SEARCHING_TARGET && nextProduct_.item_location == "HUSKY")
        {
            for(auto & t : msg->transforms)
            {
                arucoPositions_[t.fiducial_id] = arucoWorldPosition(t.transform.translation, msg->header.frame_id);
            }

            bool fiducial_found = arucoPositions_.count(nextProduct_.marker_id);
            if(fiducial_found)
            {
                // dirigirme al check pad
                plannerPub_.publish(checkPadPose_);
                sm_.signalEvent(StateMachine::Events::OBJECT_FOUND);
            }
        }

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
        if( sm_.state() == StateMachine::WAITING_ORDER )
        {
            std::string previous_location = nextProduct_.item_location;
            nextProduct_ = *product_msg;

            if( previous_location == nextProduct_.item_location &&
                arucoPositions_.count(nextProduct_.marker_id))
            {
                final_aerial_project::ProductFeedback feedbackMsg;
                feedbackMsg.marker_id = nextProduct_.marker_id;
                feedbackMsg.approximate_position = arucoPositions_[nextProduct_.marker_id];
                productFeedbackPub_.publish(feedbackMsg);
                // opcional, enviar TARGET_REACHED, no va a hacer caso
                //sm_.signalEvent(StateMachine::Events::TARGET_REACHED);
            }
            else
            {
                geometry_msgs::PoseStamped goalMsg;
                goalMsg.pose = nextProduct_.approximate_pose.pose;
                goalMsg.header.frame_id = "world";
                goalMsg.header.stamp = ros::Time::now();
                plannerPub_.publish(goalMsg);
                sm_.signalEvent(StateMachine::Events::ORDER_RECEIVED);
                arucoPositions_.clear();
            }
        }
    }

    bool poseReached(geometry_msgs::Pose currentPose, geometry_msgs::Pose targetPose)
    {
        double dx = currentPose.position.x - targetPose.position.x;
        double dy = currentPose.position.y - targetPose.position.y;
        double dz = currentPose.position.z - targetPose.position.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        double currentYaw = tf::getYaw(currentPose.orientation);
        double targetYaw  = tf::getYaw(targetPose.orientation);
        //double angleDiff = currentYaw - targetYaw;
        double angleDiff = abs(angles::shortest_angular_distance(targetYaw, currentYaw));
        return (distance < 0.5) && (angleDiff < 0.2);
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
            // espera de unos 5seg
            sm_.signalEvent(StateMachine::Events::TAKE_OFF);
        }

        if(sm_.state() == StateMachine::FLYING_TO_TARGET)
        {
            if( poseReached(msg->pose.pose, nextProduct_.approximate_pose.pose) )
            {
                // hemos llegado al target, ahora decidir qué hago
                onTargetReached(nextProduct_.item_location);
            }
        }
        else if(sm_.state() == StateMachine::FLYING_TO_CHECKPAD)
        {
            if( poseReached(msg->pose.pose, checkPadPose_.pose) )
            {
                  // hemos llegado al checkpad, publicamos la posicion de la caja
                  final_aerial_project::ProductFeedback feedbackMsg;
                  feedbackMsg.marker_id = nextProduct_.marker_id;
                  feedbackMsg.approximate_position = arucoPositions_[nextProduct_.marker_id];
                  productFeedbackPub_.publish(feedbackMsg);
                  sm_.signalEvent(StateMachine::Events::TARGET_REACHED);
            }
        }

    }

    void onTargetReached(std::string location)
    {
        if( location == "SHELVE" )
        {
            trajectoryPub_.publish(shelveTrajectory_);
            sm_.signalEvent(StateMachine::Events::TARGET_REACHED);
        }
        else if(location == "HUSKY")
        {
          trajectoryPub_.publish(huskyTrajectory_);
          sm_.signalEvent(StateMachine::Events::TARGET_REACHED);
        }
        else if(location == "END")
        {
            sm_.signalEvent(StateMachine::Events::CHARGING_REACHED);
        }
    }

    bool loop()
    {
        if( sm_.state() == StateMachine::DONE)
        {
            return true;
        }

        return false;
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

    ros::Publisher plannerPub_;
    ros::Publisher trajectoryPub_;
    ros::Subscriber frontalCameraArucoSub_;
    ros::Subscriber ventralCameraArucoSub_;

    int32_t remainingBattery_;

    // detection_threshold from parameter server
    float detectionThreshold_;

    int32_t lowBatteryThreshold_;

    nav_msgs::Odometry currentOdom_;

    StateMachine sm_;

    bool takeOffSent_;
    double takeOffHeight_;

    final_aerial_project::ProductInfo nextProduct_;

    trajectory_msgs::MultiDOFJointTrajectory shelveTrajectory_;
    trajectory_msgs::MultiDOFJointTrajectory huskyTrajectory_;

    std::map<int32_t, geometry_msgs::Point> arucoPositions_;

    geometry_msgs::PoseStamped checkPadPose_;
};




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mission_planner");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    MissionPlanner mission_planner(nh, nh_private);

    bool mission_finished = false;
    ros::Rate rate(10);
    while(ros::ok() && mission_finished == false)
    {
        rate.sleep();
        mission_finished = mission_planner.loop();
        ros::spinOnce();
    }

    if(mission_finished)
    {
        ROS_INFO("Mission Finished!!");
    }
    else
    {
        ROS_ERROR("Mission Aborted!!");
    }

    return 0;
}
