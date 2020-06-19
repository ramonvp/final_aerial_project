#include "ros/ros.h"

#include <Eigen/Eigen>
#include <random>

#include <thread>
#include <chrono>

#include "std_msgs/Int32.h"

#include "final_aerial_project/ProductInfo.h"
#include "final_aerial_project/ProductFeedback.h"

#include "nav_msgs/Odometry.h"

#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"

#define M_PI 3.14159265358979323846 /* pi */

class dispatcher_station_node
{

public:
  dispatcher_station_node() // Class constructor
  {
    ros::NodeHandle nh_;              // Public nodehandle for pub-sub
    ros::NodeHandle nh_private_("~"); // Private nodehandle for handling parameters

    // Init subscribers
    drone_sub_ = nh_.subscribe("/drone_gt", 1, &dispatcher_station_node::drone_callback, this);
    product_pos_ = nh_.subscribe("/product_feedback", 1, &dispatcher_station_node::product_callback, this);
    battery_timer_pub_ = nh_.advertise<std_msgs::Int32>("/battery_timer", 1, false);

    // Init publishers
    next_product_pub_ = nh_.advertise<final_aerial_project::ProductInfo>("/parcel_dispatcher/next_product", 1, false);

    // Init vars form param server info & others
    load_from_param_server(nh_private_);

    // Create timer to publish on battery time topic
    battery_timer_ = nh_.createTimer(ros::Duration(1.0), &dispatcher_station_node::updateBattery, this);

    current_product_dispatched = 0;
    //Init battery time
    battery_remaining_time = battery_time * 60;
    battery_empty = false;
    // Dipatch first product:
    fill_parcel_msg();
  }

private:
  // verifies the drone position and stores it on drone_position Eigen::Vector3d to later use
  void drone_callback(const geometry_msgs::Pose::ConstPtr &pose_msg)
  {
    drone_position << pose_msg->position.x, pose_msg->position.y, pose_msg->position.z;

    next_product_pub_.publish(product_msg_out_);
  }


  // Runs when product feedback is sended by the drone. Verifies that the drone is near the pase and that the
  // id it is throwing out  corresponds with the product asked.
  // If distance constraint and id are adequate, the next product request is broadcasted
  void product_callback(const final_aerial_project::ProductFeedback::ConstPtr &product_msg)
  {
    if (drone_in_place(&check_pad_position))
    {
      if (product_msg->marker_id == products_to_dispatch_id.at(current_product_dispatched))
      {
        ROS_INFO("Parcel_dispatcher node: Product Feedback received: \n Parcel location found at \n \t x:%f \n \t y:%f \n \t z:%f", product_msg->approximate_position.x, product_msg->approximate_position.y, product_msg->approximate_position.z);
        current_product_dispatched++;
        fill_parcel_msg();
      }
      else
      {
        ROS_INFO("Parcel_dispatcher node: Recalculating target position");
        fill_parcel_msg();
        // TODO: Register product found
      }
    }
  }

  // Function that check for distance between drone and check pads
  bool drone_in_place(Eigen::Vector3d *position)
  {
    Eigen::Vector3d dist;

    dist = drone_position - *position;
    return dist.norm() <= detection_threshold;
  }

  // Controls the requested parcel. IF parcel is in shelve publishes the pose of a point near the shell.
  // If the parcel is over the husky, publishes the husky pose with added uncertainty
  // If Mision has ended publishes the pose of the charging base
  void fill_parcel_msg()
  {

    if (current_product_dispatched < number_of_products_to_dispatch)
    {
      geometry_msgs::PoseWithCovariance zeroPose;
      product_msg_out_.approximate_pose = zeroPose;

      product_msg_out_.item_location = products_to_dispatch_location.at(current_product_dispatched);
      product_msg_out_.marker_id = products_to_dispatch_id.at(current_product_dispatched);

      if (products_to_dispatch_location.at(current_product_dispatched).compare("SHELVE") == 0)
      {
        product_msg_out_.approximate_pose = shelve_pose;
        ROS_INFO("Parcel_dispatcher node:  Parcel in Shelve Requested");
      }

      if (products_to_dispatch_location.at(current_product_dispatched).compare("HUSKY") == 0)
      {
        product_msg_out_.approximate_pose = alter_husky_pose(husky_pose);
        ROS_INFO("Parcel_dispatcher node: Parcel in Husky Requested");
      }

      if (products_to_dispatch_location.at(current_product_dispatched).compare("END") == 0)
      {
        product_msg_out_.approximate_pose = charging_pad_pose;
        ROS_INFO("Parcel_dispatcher node: Mission has ended: Charging pad pose has been published");
      }

      next_product_pub_.publish(product_msg_out_);
    }
  }

  // function that modifies the husky pose to add noise
  geometry_msgs::PoseWithCovariance alter_husky_pose(geometry_msgs::PoseWithCovariance pose)
  {
    std::normal_distribution<double> distribution(0.0, 2.0); // 2d sigma cov on x-z hardcoded to 2m

    pose.covariance[0] = 4.0;
    pose.covariance[7] = 4.0; // 2d sigma cov on x-z hardcoded to 2m -> sigma sqr = 4m

    pose.pose.position.x += distribution(generator);
    pose.pose.position.y += distribution(generator);

    return pose;
  }

  // function that updates the remaining battery time and calls the check on battery charge
  void updateBattery(const ros::TimerEvent &event)
  {

    if (battery_remaining_time > 0 && drone_position[2] > 0.2)
    {
      battery_remaining_time -= 1;
    }
    else if (battery_remaining_time <= 0)
    {
      battery_empty = true;
      battery_remaining_time = 0;
    }

    int_msg_.data = battery_remaining_time;
    battery_timer_pub_.publish(int_msg_);

    checkBatteryCharged();
  }

  // function that checks if the drone is close by the location of the charging pad, thus charging
  void checkBatteryCharged()
  {
    if (drone_in_place(&charging_pad_position))
    {
      battery_remaining_time = battery_time * 60;
      battery_empty = false;
    }
  }

  // Handy function for initialising covariance matrices from parameters
  void load_from_param_server(ros::NodeHandle &nh_private_)
  {

    // get detection threshold
    double detection_threshold_default = 1.0;

    if (nh_private_.hasParam("/parcel_dispatcher/detection_threshold"))
    {
      nh_private_.getParam("/parcel_dispatcher/detection_threshold", detection_threshold);
    }
    else
    {
      detection_threshold = detection_threshold_default;
      ROS_INFO("Parcel_dispatcher node: detection_threshold not set in the parameter server using a default one");
    }

    // get targets info
    nh_private_.getParam("/parcel_dispatcher/numberOfTargets", number_of_products_to_dispatch);
    products_to_dispatch_id.resize(number_of_products_to_dispatch);
    products_to_dispatch_location.resize(number_of_products_to_dispatch);

    nh_private_.getParam("/parcel_dispatcher/marker_id", products_to_dispatch_id);
    nh_private_.getParam("/parcel_dispatcher/item_location", products_to_dispatch_location);

    // get interesting places info
    std::vector<double> dummy3DVector{0.0, 0.0, 0.0};
    std::vector<double> dummy7DVector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    nh_private_.getParam("/parcel_dispatcher/check_pad_position", dummy3DVector);

    check_pad_position << dummy3DVector.at(0), dummy3DVector.at(1), dummy3DVector.at(2);

    nh_private_.getParam("/parcel_dispatcher/charging_pad_pose", dummy7DVector);
    charging_pad_pose = pose_with_covariance_from_7D_vec(dummy7DVector);
    charging_pad_position  << dummy7DVector.at(0), dummy7DVector.at(1), dummy7DVector.at(2);

    nh_private_.getParam("/parcel_dispatcher/shelve_pose", dummy7DVector);
    shelve_pose = pose_with_covariance_from_7D_vec(dummy7DVector);

    nh_private_.getParam("/parcel_dispatcher/husky_pose", dummy7DVector);
    husky_pose = pose_with_covariance_from_7D_vec(dummy7DVector);

    //get battery info
    if (nh_private_.hasParam("/parcel_dispatcher/battery_time"))
    {
      nh_private_.getParam("/parcel_dispatcher/battery_time", battery_time);
    }
  }

  geometry_msgs::PoseWithCovariance pose_with_covariance_from_7D_vec(std::vector<double> poseVec)
  {
    geometry_msgs::PoseWithCovariance geom_pose;

    geom_pose.pose.position.x = poseVec.at(0);
    geom_pose.pose.position.y = poseVec.at(1);
    geom_pose.pose.position.z = poseVec.at(2);

    double quatnorm = 0.0;
    for (int ii = 3; ii < 7; ii++)
    {
      quatnorm += poseVec.at(ii) * poseVec.at(ii);
    }
    quatnorm = std::sqrt(quatnorm);

    geom_pose.pose.orientation.x = poseVec.at(3) / quatnorm;
    geom_pose.pose.orientation.y = poseVec.at(4) / quatnorm;
    geom_pose.pose.orientation.z = poseVec.at(5) / quatnorm;
    geom_pose.pose.orientation.w = poseVec.at(6) / quatnorm;

    return geom_pose;
  }

protected:
  // Subscriber objects
  ros::Subscriber drone_sub_;
  ros::Subscriber product_pos_;
  ros::Timer battery_timer_;

  // Publisher objects
  ros::Publisher next_product_pub_;
  ros::Publisher battery_timer_pub_;

  // Message objects
  final_aerial_project::ProductInfo product_msg_out_;
  std_msgs::Int32 int_msg_;

  // Pose of elements
  geometry_msgs::PoseWithCovariance husky_pose;
  geometry_msgs::PoseWithCovariance shelve_pose;
  geometry_msgs::PoseWithCovariance charging_pad_pose;
  geometry_msgs::PoseWithCovariance check_pad_pose; // not sure if needed

  // bases info to load from parameter server
  Eigen::Vector3d check_pad_position;
  Eigen::Vector3d charging_pad_position;

  // Products to dispatch
  int number_of_products_to_dispatch;
  std::vector<int> products_to_dispatch_id;
  std::vector<std::string> products_to_dispatch_location;
  int current_product_dispatched;

  // Battery topic related params
  int battery_time;
  int battery_remaining_time;
  bool battery_empty;

  // detection_threshold from parameter server
  float detection_threshold;

  // drone position to read from gt topic
  Eigen::Vector3d drone_position;

  // to add noise to the husky pose
  std::default_random_engine generator;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parcel_dispatcher");

  dispatcher_station_node dispatcher_node;

  ros::spin();
}
