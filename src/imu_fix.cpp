#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

static ros::Publisher imu_pub;

static double orientation_cov[3];

void imu_callback(const sensor_msgs::ImuConstPtr & msg)
{
    sensor_msgs::Imu fixed_msg = *msg;
    fixed_msg.orientation_covariance[0] = orientation_cov[0];
    fixed_msg.orientation_covariance[1] = 0;
    fixed_msg.orientation_covariance[2] = 0;

    fixed_msg.orientation_covariance[3] = 0;
    fixed_msg.orientation_covariance[4] = orientation_cov[1];
    fixed_msg.orientation_covariance[5] = 0;

    fixed_msg.orientation_covariance[6] = 0;
    fixed_msg.orientation_covariance[7] = 0;
    fixed_msg.orientation_covariance[8] = orientation_cov[2];
    imu_pub.publish(fixed_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_fix_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_params("~");

    nh_params.param<double>("orientation_cov_roll", orientation_cov[0], 0.01745);
    nh_params.param<double>("orientation_cov_pitch", orientation_cov[1], 0.01745);
    nh_params.param<double>("orientation_cov_yaw", orientation_cov[2], 0.15708);

    ros::Subscriber imu_sub = nh.subscribe("/firefly/imu", 1, &imu_callback);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/firefly/imu_fixed", 10);

    ros::spin();

    return 0;
}
