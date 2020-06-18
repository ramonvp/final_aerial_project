#include "ros/ros.h"

#include <thread>
#include <chrono>
#include <random>

#include <std_srvs/Empty.h>

#include "sensor_msgs/Imu.h"

#include "nav_msgs/Odometry.h"

#include "geometry_msgs/TwistWithCovarianceStamped.h"


# define M_PI  3.14159265358979323846  /* pi */

class fake_twist_odometer
{

public:
  fake_twist_odometer() // Class constructor
  {
    ros::NodeHandle nh_; // Public nodehandle for pub-sub
    ros::NodeHandle nh_private_("~"); // Private nodehandle for handling parameters

    // Init subscribers
    odom_sub_ = nh_.subscribe("/ground_truth_odometry", 1, &fake_twist_odometer::odom_callback, this);

    // Init publishers
    twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/firefly/fake_twist", 5, false);

    twist_out_.header.frame_id = "firefly/base_link2";

    // Init covariances grabbed from the parameter server
    load_cov_from_param_server(nh_private_);
  
  }

 
  void odom_callback(nav_msgs::Odometry odom_msg)
  {
    double randomNumber;
    std::normal_distribution<double> distribution(0.0,1.0);
    twist_out_.header.stamp = odom_msg.header.stamp;
    twist_out_.twist.twist = odom_msg.twist.twist;

    randomNumber = distribution(generator)* sqrt(linVel_covar);
    twist_out_.twist.twist.linear.x  += randomNumber;

    randomNumber = distribution(generator)* sqrt(linVel_covar);
    twist_out_.twist.twist.linear.y  += randomNumber;

    randomNumber = distribution(generator)* sqrt(linVel_covar);
    twist_out_.twist.twist.linear.z  += randomNumber;

    randomNumber = distribution(generator)* sqrt(angVel_covar);
    twist_out_.twist.twist.angular.x  += randomNumber;

    randomNumber = distribution(generator)* sqrt(angVel_covar);
    twist_out_.twist.twist.angular.y  += randomNumber;

    randomNumber = distribution(generator)* sqrt(angVel_covar);
    twist_out_.twist.twist.angular.z  += randomNumber;
    

    // publish msg
    twist_pub_.publish(twist_out_);
    }
    
  // Handy function for initialising covariance matrices from parameters
  void load_cov_from_param_server(ros::NodeHandle &nh_private_)
  {

    // Create the vectors to store the covariance matrix arrays

    double linVel_covar_default = 0.02;
    double angVel_covar_default = 0.05;

    // Grab the parameters and populate the vectors
    if (nh_private_.hasParam("/firefly/fake_twist/linVel_covariance")){
        nh_private_.getParam("/firefly/fake_twist/linVel_covariance", linVel_covar);
        ROS_INFO("Fake Covariance for fake twist (linVel) odometer fetched from parameter Server");
    }else{
        linVel_covar = linVel_covar_default;
        ROS_INFO("Fake Covariance for fake twist (linVel) odometer not set in the parameter server, using a default one");
    }

    if (nh_private_.hasParam("/firefly/fake_twist/angVel_covariance")){
        nh_private_.getParam("/firefly/fake_twist/angVel_covariance", angVel_covar);
        ROS_INFO("Fake Covariance for fake twist (angVel) odometer fetched from parameter Server");
    }else{
        angVel_covar = angVel_covar_default;
        ROS_INFO("PFake Covariance for fake twist (angVel) odometer not set in the parameter server, using a default one");
    }

    // Iterate through each vector and populate the covariance message fields

        
    for (int i = 0; i < 3; i++){
            twist_out_.twist.covariance[6*i+i] = linVel_covar;
            twist_out_.twist.covariance[6*(i+3)+i+3] = angVel_covar;
    }

    // TO DO: allow non diagonal covariances non uniform covariances...
  }


protected:
  // Subscriber objects
  ros::Subscriber odom_sub_;
  // Publisher objects
  ros::Publisher twist_pub_;

  // Message objects
  geometry_msgs::TwistWithCovarianceStamped twist_out_;

  // covariance info
  double linVel_covar;
  double angVel_covar;

  std::default_random_engine generator;




};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_twist_odometer");
  
  fake_twist_odometer fake_odometer;

  ros::spin();
}
