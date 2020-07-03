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
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3Stamped.h>

std::ostream& operator<< (std::ostream& os, const tf::Stamped<tf::Vector3> & v)
{
    os << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
    return os;
}


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
      LOW_BATTERY,
      CHARGING_REACHED
  };

  StateMachine() : state_{INITIALIZING}
  {

  }

  const char * getStateName(State state)
  {
      switch(state)
      {
          case INITIALIZING: return "INITIALIZING";
          case WAITING_ORDER: return "WAITING_ORDER";
          case FLYING_TO_TARGET: return "FLYING_TO_TARGET";
          case FLYING_TO_RECHARGE: return "FLYING_TO_RECHARGE";
          case FLYING_TO_CHECKPAD: return "FLYING_TO_CHECKPAD";
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
  }

  void StateFlyingToTarget(Events event)
  {
      if( event == Events::TARGET_REACHED )
      {
          transitionTo(SEARCHING_TARGET);
      }
      else if( event == Events::LOW_BATTERY )
      {
          transitionTo(FLYING_TO_RECHARGE);
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
      else if( event == Events::LOW_BATTERY )
      {
          transitionTo(FLYING_TO_RECHARGE);
      }
  }

  void StateFlyingToRecharge(Events event)
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
          case FLYING_TO_RECHARGE: StateFlyingToRecharge(event); break;
          case FLYING_TO_CHECKPAD: StateFlyingToCheckpad(event); break;
          case SEARCHING_TARGET: StateSearchingTarget(event); break;
          case DONE:
          default:
              break;
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
        takeOffHeight_{1.0},
        distanceThreshold_{0.5},
        yawThreshold_{0.2},
        poseReachedTrigger_{10},
        tf_listener_(ros::Duration(10.0)),
        global_frame_{"world"},
        base_frame_{"firefly/base_link"},
        transform_tolerance_{0.3}
    {
          nextProduct_.item_location = "UNKNOWN";
          nextProduct_.marker_id = -1;

          dronePoseSub_ = nh_.subscribe("odom", 1, &MissionPlanner::odomCallback, this);
          batterySub_ = nh_.subscribe("/firefly/battery_timer", 1, &MissionPlanner::batteryCallback, this);
          productFeedbackPub_ = nh_.advertise<final_aerial_project::ProductFeedback>("/firefly/product_feedback", 1);
          productInfoSub_ = nh_.subscribe("/parcel_dispatcher/next_product", 1, &MissionPlanner::dispatcherCallback, this);
          poseCommandPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);

          plannerPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

          trajectoryPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);


          frontalCameraArucoSub_ = nh_.subscribe("/frontal/fiducial_transforms", 1, &MissionPlanner::frontalArucoCallback, this);
          ventralCameraArucoSub_ = nh_.subscribe("/ventral_search/fiducial_transforms", 1, &MissionPlanner::ventralArucoCallback, this);

          // fill shelve trajectory
          shelveTrajectory_.points.push_back( createTrajPoint(15,3,1, -M_PI/2) );
          shelveTrajectory_.points.push_back( createTrajPoint(15,3,3.7, -M_PI/2) );
          shelveTrajectory_.points.push_back( createTrajPoint(17,3,3.7, -M_PI/2) );
          shelveTrajectory_.points.push_back( createTrajPoint(17,3,1.0, -M_PI/2) );

          // fill huskyTrajectory_
          huskyTrajectory_.points.push_back( createTrajPoint(7.5,25.0,1.0, M_PI) );
          huskyTrajectory_.points.push_back( createTrajPoint(5.0,25.0,1.0, -M_PI/2) );
          huskyTrajectory_.points.push_back( createTrajPoint(5.0,23.5,1.0, 0) );
          huskyTrajectory_.points.push_back( createTrajPoint(7.5,23.5,1.0, M_PI/2) );

          double robot_clearance_radius_ {1.0}; // should match the config of voxblox

          // Read the coordinates from parameter server of the CheckPad
          nh_.param("/parcel_dispatcher/detection_threshold", detectionThreshold_, detectionThreshold_);

          checkPadPose_.header.frame_id = "world";
          std::vector<double> dummy3DVector{0.0, 0.0, 0.0};
          nh_.getParam("/parcel_dispatcher/check_pad_position", dummy3DVector);
          checkPadPose_.pose.position.x = dummy3DVector.at(0);
          checkPadPose_.pose.position.y = dummy3DVector.at(1);
          //checkPadPose_.pose.position.z = detectionThreshold_;//dummy3DVector.at(2);
          checkPadPose_.pose.position.z = dummy3DVector.at(2);
          if(checkPadPose_.pose.position.z < robot_clearance_radius_)
          {
               checkPadPose_.pose.position.z += detectionThreshold_ - 0.2;
          }
          //checkPadPose_.pose.position.x = 21.82;
          //checkPadPose_.pose.position.y = 3.13;
          //checkPadPose_.pose.position.z = 0.055;
          checkPadPose_.pose.orientation.w = 1.0;

          // Get the charging pad position from parameter server
          std::vector<double> dummy7DVector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          nh_.getParam("/parcel_dispatcher/charging_pad_pose", dummy7DVector);
          chargingPadPose_.pose.position.x = dummy7DVector.at(0);
          chargingPadPose_.pose.position.y = dummy7DVector.at(1);
          chargingPadPose_.pose.position.z = dummy7DVector.at(2);
          chargingPadPose_.pose.orientation.w = 1.0;
          if(chargingPadPose_.pose.position.z < robot_clearance_radius_)
          {
               chargingPadPose_.pose.position.z += detectionThreshold_ - 0.2;
          }

          nh_params_.param("distance_threshold", distanceThreshold_, distanceThreshold_);
          nh_params_.param("yaw_threshold", yawThreshold_, yawThreshold_);
          nh_params_.param("low_battery_secs", lowBatteryThreshold_, lowBatteryThreshold_);


          ROS_INFO("[MissionPlanner] Ready");
    }


    trajectory_msgs::MultiDOFJointTrajectoryPoint createTrajPoint(double x, double y, double z, double yaw)
    {
        trajectory_msgs::MultiDOFJointTrajectoryPoint p;
        geometry_msgs::Transform t;
        t.translation.x = x;
        t.translation.y = y;
        t.translation.z = z;
        tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, yaw);
        tf::quaternionTFToMsg(quat, t.rotation);
        p.transforms.push_back(t);
        return p;
    }

    void frontalArucoCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg)
    {
        if( msg->transforms.size() == 0) return;

        if( sm_.state() == StateMachine::SEARCHING_TARGET && nextProduct_.item_location == "SHELVE")
        {
            observeArucoMarkers(msg);
        }
    }

    void ventralArucoCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg)
    {
        if( msg->transforms.size() == 0) return;

        if( sm_.state() == StateMachine::SEARCHING_TARGET && nextProduct_.item_location == "HUSKY")
        {
            observeArucoMarkers(msg);
        }
    }

    void observeArucoMarkers(const fiducial_msgs::FiducialTransformArray::ConstPtr & msg)
    {
        for(auto & t : msg->transforms)
        {
            geometry_msgs::Vector3Stamped vector;
            vector.header = msg->header;
            // FIX: gazebo publishes the images in a wrong frame, manual correction needed
            if(msg->header.frame_id == "firefly/vi_sensor/camera_left_link")
            {
                vector.header.frame_id = "firefly/vi_sensor/camera_left_optical_link";
            }



            vector.vector = t.transform.translation;
            geometry_msgs::Point p = arucoWorldPosition(vector);
            arucoPositions_[t.fiducial_id] = p;
            //printf("Aruco %d found at %f,%f,%f\n", t.fiducial_id, p.x, p.y, p.z);

        }

        bool fiducial_found = arucoPositions_.count(nextProduct_.marker_id);
        if(fiducial_found)
        {
            // go to the check pad
            fly_to(checkPadPose_.pose);
            sm_.signalEvent(StateMachine::Events::OBJECT_FOUND);
        }
    }

    geometry_msgs::Vector3Stamped transform_dir(const geometry_msgs::Vector3Stamped & dir)
    {
        tf::Stamped<tf::Vector3> in;
        tf::Stamped<tf::Vector3> out;

        tf::Stamped<tf::Pose> global_pose;
        global_pose.setIdentity();
        tf::Stamped<tf::Pose> aruco_pose_base_link;
        aruco_pose_base_link.setIdentity();
        aruco_pose_base_link.frame_id_ = base_frame_;
        aruco_pose_base_link.stamp_ = in.stamp_;


        tf::vector3StampedMsgToTF(dir, in);

        try {
            tf_listener_.transformVector(base_frame_, in, out);

            aruco_pose_base_link.setOrigin(out);
            tf_listener_.transformPose(global_frame_, aruco_pose_base_link, global_pose);
        } catch (tf::LookupException& ex) {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up camera to robot "
                                    "tf: %s\n",
                               ex.what());
            return {};
        } catch (tf::ConnectivityException& ex) {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up camera to robot tf: %s\n",
                               ex.what());
            return {};
        } catch (tf::ExtrapolationException& ex) {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up camera to robot tf: %s\n",
                               ex.what());
            return {};
        }

        geometry_msgs::Vector3Stamped msg;
        msg.vector.x = global_pose.getOrigin().x();
        msg.vector.y = global_pose.getOrigin().y();
        msg.vector.z = global_pose.getOrigin().z();

        return msg;
    }

    geometry_msgs::Point arucoWorldPosition(const geometry_msgs::Vector3Stamped & vector)
    {
        geometry_msgs::Point p;

        // TF lookup of aruco pos
        geometry_msgs::Vector3Stamped vector_world = transform_dir(vector);

        p.x = vector_world.vector.x;
        p.y = vector_world.vector.y;
        p.z = vector_world.vector.z;

        return p;
    }


    void batteryCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        remainingBattery_ = msg->data;

        // If we have still enough battery, we are fine
        if( remainingBattery_ > lowBatteryThreshold_ )
            return;

        if( sm_.state() != StateMachine::FLYING_TO_RECHARGE)
        {
            // Go back to charging pad
            ROS_WARN("[MissionPlanner] Low battery: %d secs", remainingBattery_);

            // TODO: If we are flying to CheckPad and we are
            // very close, maybe wait a little bit
            if(sm_.state() == StateMachine::FLYING_TO_CHECKPAD)
            {
               double distance = euclideanDistance(currentOdom_.pose.pose.position, checkPadPose_.pose.position);
               if(distance < 12.0) return;
            }

            // TODO: currently disabled, check state machine loops
            //sm_.signalEvent(StateMachine::Events::LOW_BATTERY);
            //plannerPub_.publish(chargingPadPose_);
        }
    }

    void dispatcherCallback(const final_aerial_project::ProductInfo::ConstPtr &product_msg)
    {
        if( sm_.state() == StateMachine::WAITING_ORDER )
        {
            ROS_INFO("[Mission] Order received, location: %s  marker_id: %ld at (%g,%g,%g)",
                     product_msg->item_location.c_str(),
                     product_msg->marker_id,
                     product_msg->approximate_pose.pose.position.x,
                     product_msg->approximate_pose.pose.position.y,
                     product_msg->approximate_pose.pose.position.z);

            if( product_msg->item_location == nextProduct_.item_location &&
                product_msg->marker_id == nextProduct_.marker_id)
            {
                ROS_WARN("[Mission] Waiting order, but previous order was received. Discarding...");
                return;
            }
            std::string previous_location = nextProduct_.item_location;
            nextProduct_ = *product_msg;

            // we don't want to fly so close to the ground
            if(nextProduct_.approximate_pose.pose.position.z < detectionThreshold_)
            {
                nextProduct_.approximate_pose.pose.position.z = detectionThreshold_;
            }

            fly_to(nextProduct_.approximate_pose.pose);
            sm_.signalEvent(StateMachine::Events::ORDER_RECEIVED);
            arucoPositions_.clear();
#if 0
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
                fly_to(nextProduct_.approximate_pose.pose);
                sm_.signalEvent(StateMachine::Events::ORDER_RECEIVED);
                arucoPositions_.clear();
            }
#endif
        }
    }

    double euclideanDistance(const geometry_msgs::Point & a, const geometry_msgs::Point & b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    bool poseReached(geometry_msgs::Pose currentPose, geometry_msgs::Pose targetPose, double & pending_dist, double & pending_yaw)
    {
        double distance   = euclideanDistance(currentPose.position, targetPose.position);
        double currentYaw = tf::getYaw(currentPose.orientation);
        double targetYaw  = tf::getYaw(targetPose.orientation);
        pending_yaw = angles::shortest_angular_distance(targetYaw, currentYaw);
        double angleDiff  = abs(pending_yaw);

        pending_dist = distance;
        pending_yaw  = angleDiff;

        return (distance < distanceThreshold_) && (angleDiff < yawThreshold_);
    }

    bool ifFlyingSomewhere() const
    {
        return sm_.state() == StateMachine::FLYING_TO_TARGET ||
               sm_.state() == StateMachine::FLYING_TO_CHECKPAD;
    }

    void takeOff()
    {
        ROS_INFO("[MissionPlanner] Take Off!");
        geometry_msgs::PoseStamped takeOffMsg;
        takeOffMsg.header = currentOdom_.header;
        takeOffMsg.pose = currentOdom_.pose.pose;
        takeOffMsg.pose.position.z = takeOffHeight_;
        poseCommandPub_.publish(takeOffMsg);
        takeOffSent_ = true;
        // wait for a few seconds to be airborne
        ros::Rate r(10);
        int counter = 0;
        while(counter < 50) // about 5segs
        {
            ros::spinOnce();
            r.sleep();
            counter++;
        }
        sm_.signalEvent(StateMachine::Events::TAKE_OFF);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        static int counter = 0;
        static int poseReachedCounter = 0;

        ROS_INFO_ONCE("[MissionPlanner] First odom message received");
        currentOdom_ = *msg;

        if(!takeOffSent_)
        {
            takeOff();
            return;
        }

        if( !ifFlyingSomewhere() )
            return;

        // So we are here waiting for the drone to reach some programmed point
        double pending_dist, pending_yaw;
        if( poseReached(msg->pose.pose, plannerTargetPose_, pending_dist, pending_yaw) )
        {
            // require confirmation that we are stable
            poseReachedCounter++;
            if(poseReachedCounter < poseReachedTrigger_) return;
            poseReachedCounter = 0;

            if(sm_.state() == StateMachine::FLYING_TO_TARGET)
            {
                // we reached the approximate location of the item we have to look for
                ROS_INFO("[Mission] Reached target destination: %s", nextProduct_.item_location.c_str());
                onTargetReached(nextProduct_.item_location);
            }
            else if(sm_.state() == StateMachine::FLYING_TO_CHECKPAD)
            {
                // hemos llegado al checkpad, publicamos la posicion de la caja
                ROS_INFO("[Mission] CheckPad reached");
                final_aerial_project::ProductFeedback feedbackMsg;
                feedbackMsg.marker_id = nextProduct_.marker_id;
                feedbackMsg.approximate_position = arucoPositions_[nextProduct_.marker_id];
                productFeedbackPub_.publish(feedbackMsg);
                sm_.signalEvent(StateMachine::Events::TARGET_REACHED);
            }
        }
        else
        {
            poseReachedCounter = 0;
            if(counter%10 == 0)
            {
                ROS_INFO("[Mission] Flying to target, pending dist: %.02f  yaw: %.02f",
                         pending_dist, pending_yaw);
            }
        }
        counter++;
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

    bool done()
    {
        static int counter = 0;

        if(counter%10 == 0)
        {
            ROS_INFO("[Mission] Current state: %s", sm_.getStateName(sm_.state()));
        }

        counter++;

        return sm_.state() == StateMachine::DONE;
    }

    void fly_to(const geometry_msgs::Pose & pose)
    {
        geometry_msgs::PoseStamped goalMsg;
        goalMsg.pose = pose;
        goalMsg.header.frame_id = "world";
        goalMsg.header.stamp = ros::Time::now();
        plannerPub_.publish(goalMsg);

        plannerTargetPose_ = pose;
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_params_;

    ros::Subscriber dronePoseSub_;
    ros::Subscriber batterySub_;
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

    double distanceThreshold_;
    double yawThreshold_;
    int poseReachedTrigger_;

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
    geometry_msgs::PoseStamped chargingPadPose_;

    tf::TransformListener tf_listener_;
    std::string global_frame_;
    std::string base_frame_;
    double transform_tolerance_;
    // the target pose sent to the planner
    geometry_msgs::Pose plannerTargetPose_;
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
        mission_finished = mission_planner.done();
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
