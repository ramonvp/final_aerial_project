#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/common.h>

ros::Publisher velocity_pub;
std::vector<Eigen::Vector2d> wp_list;
Eigen::Vector2d setpoint;
std::string mission_file_;
nav_msgs::Odometry latest_pose;
double thr_rotation_error_;
double rotation_speed_;
double linear_speed_;

void poseCallback(const nav_msgs::OdometryConstPtr &msg)
{
    ROS_INFO_ONCE("First Pose msg received ");
    latest_pose = *msg; // Handle pose measurements.
}

bool loadWPs(const std::string &filename)
{
    //CHECK(!filename.empty());

    // Open and check the file
    std::fstream file;
    file.open(filename, std::fstream::in);
    if (!file.is_open())
    {
        ROS_ERROR("Could not open file to load graph points: ", filename);
        return false;
    }
    else
    {
        std::string line;
        ROS_INFO("[Husky Node] Load mission file.");
        int64_t id = 0;
        while (getline(file, line))
        {
            std::istringstream line_stream(line);

            double x, y;
            line_stream >> x >> y;
            Eigen::Vector2d wp{x, y};
            wp_list.push_back(wp);
            ++id;
        }
        return true;
    }
}

bool computeVelCommands(int wp_index, geometry_msgs::Twist &vel_commands)
{
    double dist = sqrt((setpoint[0] - latest_pose.pose.pose.position.x) * (setpoint[0] - latest_pose.pose.pose.position.x) +
                       (setpoint[1] - latest_pose.pose.pose.position.y) * (setpoint[1] - latest_pose.pose.pose.position.y));
    Eigen::Vector2d direction{(setpoint[0] - latest_pose.pose.pose.position.x) / dist, (setpoint[1] - latest_pose.pose.pose.position.y) / dist};

    vel_commands.linear.x = direction[0] * linear_speed_;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_params("~");

    ROS_INFO("Initializing husky controller node...");

    ros::Subscriber pose_sub = nh.subscribe("odometry/filtered", 1, &poseCallback);
    ros::Publisher vel_command_pub = nh.advertise<geometry_msgs::Twist>("twist_marker_server/cmd_vel", 1);

    nh_params.param("mission_file", mission_file_, mission_file_);
    nh_params.param("thr_rotation_error", thr_rotation_error_, 0.1);
    nh_params.param("rotation_speed", rotation_speed_, 1.0);
    nh_params.param("linear_speed", linear_speed_, 1.0);

    //Load husky wps
    loadWPs(mission_file_);
    setpoint = wp_list[0];
    int wp_index = 0;
    double distance;
    ros::Rate r(10);
    ros::Duration(5.0).sleep();
    while (ros::ok())
    {
        ros::spinOnce();

        distance = sqrt((setpoint[0] - latest_pose.pose.pose.position.x) * (setpoint[0] - latest_pose.pose.pose.position.x) +
                        (setpoint[1] - latest_pose.pose.pose.position.y) * (setpoint[1] - latest_pose.pose.pose.position.y));
        if (distance < 0.5)
        {

            if (wp_index < wp_list.size())
            {
                setpoint = wp_list[wp_index];
                wp_index++;
            }
            else
            {
                setpoint = wp_list[0];
                wp_index = 0;
            }
        }

        geometry_msgs::Twist velocity_cmd;
        computeVelCommands(wp_index, velocity_cmd);

        vel_command_pub.publish(velocity_cmd);
        r.sleep();
    }

    return 0;
}