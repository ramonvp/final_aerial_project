#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

ros::Publisher rpyrt_command_pub;
ros::Publisher pose_pub;

nav_msgs::Odometry last_odom_msg;
bool sent_disable_msg = true;
int enable_button = 4;
bool enabled_button_pressed = false;
geometry_msgs::PoseStamped target_pose;

double linear_x {0.0};
double linear_y {0.0};
double linear_z {0.0};
double angular_w {0.0};

void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  ROS_DEBUG("Odom received");
  last_odom_msg = *odom_msg;
}

double scale_x = 0.5;
double scale_y = 0.5;
double scale_z = 1.0;
double scale_yaw = 0.5;

void joy_callback(const sensor_msgs::JoyConstPtr& joy_msg)
{
    ROS_DEBUG("Joy received");

    enabled_button_pressed = joy_msg->buttons.size() > enable_button &&
                             joy_msg->buttons[enable_button];

    if (enabled_button_pressed)
    {
        linear_x  = joy_msg->axes[1] * scale_x;
        linear_y  = joy_msg->axes[0] * scale_y;
        linear_z  = joy_msg->axes[3] * scale_z;
        angular_w = joy_msg->axes[2] * scale_yaw;
    }
    else
    {
        // When enable button is released, immediately send a single no-motion command
        // in order to stop the drone with whatever current position it is.
        if (!sent_disable_msg)
        {
            target_pose.pose = last_odom_msg.pose.pose;
            pose_pub.publish(target_pose);
            sent_disable_msg = true;

            linear_x = 0.0;
            linear_y = 0.0;
            linear_z = 0.0;
            angular_w = 0.0;
        }
    }

    ROS_DEBUG_STREAM("linear_x : " << linear_x << ", linear_y: " << linear_y << ", linear_z: " << linear_z << ", angular_w: " << angular_w);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "teleop_firefly");
    ros::NodeHandle nh;
    ros::NodeHandle nh_params("~");

    nh_params.param<double>("scale_x", scale_x, 1.0);
    nh_params.param<double>("scale_y", scale_y, 1.0);
    nh_params.param<double>("scale_z", scale_z, 1.0);
    nh_params.param<double>("scale_yaw", scale_yaw, 1.0);

    //ros::Publisher rpyrt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>("/firefly/command/roll_pitch_yawrate_thrust", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);

    ros::Subscriber odom_sub  = nh.subscribe("/firefly/ground_truth/odometry", 1, &odom_callback);
    ros::Subscriber joy_sub  = nh.subscribe("/joy", 1, &joy_callback);

    double dt = 1/10.0;
    ros::Rate r(1/dt);

    while(ros::ok())
    {
        ros::spinOnce();

        if(enabled_button_pressed)
        {
            if(sent_disable_msg)
            {
              target_pose.pose = last_odom_msg.pose.pose;
            }

            target_pose.pose.position.x += dt * linear_x;
            target_pose.pose.position.y += dt * linear_y;
            target_pose.pose.position.z += dt * linear_z;

            double yaw = tf::getYaw(target_pose.pose.orientation);
            yaw += dt * angular_w;
            target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            pose_pub.publish(target_pose);
            sent_disable_msg = false;
        }

        r.sleep();
    }

    return 0;
}
