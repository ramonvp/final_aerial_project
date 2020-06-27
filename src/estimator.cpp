#include "estimator.h"
#include <std_msgs/Empty.h>

EstimatorNode::EstimatorNode(const ros::NodeHandle & nh)
  : nh_(nh)
{
  /* Read parameters from the parameter server */
  ROS_INFO ("Reading Parameters from the parameter server"); 

  readPars(nh_);

  ROS_INFO ("Creating data structures for the Kalman Filter"); 

   /* BEGIN: Instantiate here the matrices and vectors of the Kalman filter*/

  // check documentation
  const int n = 6; // system states
  const int m = 3; // system inputs
  const int p = 3; // sensor measurements

  x_hat.resize(n);

  u.resize(m);
  y_aruco.resize(p);
  y_twist.resize(p);

  P_hat.resize(n,n);

  A.resize(n,n);
  B.resize(n,m);
  C_aruco.resize(p,n);
  C_twist.resize(p,n);

  Q.resize(n,n);

  R_aruco.resize(p,p);
  R_twist.resize(p,p);

  L.resize(n,p);

   
  /* END: Instantiate here the matrices and vectors of the Kalman filter*/
  
  /* BEGIN: Set the constants C R*/
  ROS_INFO ("Set constants");

  C_aruco << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0;

  C_twist << 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

  R_aruco << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

  R_aruco = sigma_sqr_aruco*R_aruco;

  R_twist << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

  R_twist = sigma_sqr_twist*R_twist;

  gravity_w = tf::Vector3(0.,0.,-9.8);

  /* END: Set the constants*/

    /* BEGIN: Set the initial conditions: P_hat0*/
  ROS_INFO ("Set the initial conditions "); 

  P_hat << 1, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0,
           0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 1;

  P_hat = sigma_sqr_P0*P_hat;

  x_hat << 0, 0, 0, 0, 0, 0; // reset in calibration but is ok to maintain it here too
 
  /* END: Set the initial conditions*/

  ROS_INFO ("Creating subscribers, publisher and timer"); 

  // subscriber
  //ground_truth_pose_sub_= nh_.subscribe("/firefly/ground_truth/pose_with_covariance", 1, &EstimatorNode::GroundTruthPoseWithCovarianceStampedCallback, this);
  pose_sub_             = nh_.subscribe("/firefly/aruco_pose", 1, &EstimatorNode::PoseWithCovarianceStampedCallback, this);
  imu_sub_              = nh_.subscribe("/firefly/imu", 1, &EstimatorNode::ImuCallback, this);
  twist_sub_            = nh_.subscribe("/firefly/fake_twist", 1, &EstimatorNode::TwistCallback, this);
  // publisher
  odometry_pub_  = nh_.advertise<nav_msgs::Odometry>("/firefly/odom_filtered", 1);

  // timer
  time_reference = ros::WallTime::now(); 
}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::predict ()
{
  //publish your data
  //ROS_INFO("Publishing ...");

  ros::WallTime time_reference_new = ros::WallTime::now();
  double dT = (time_reference_new-time_reference).toSec();

  if(dT < 0)
  {
    ROS_ERROR("dT is NEGATIVE!");
    return;
  }

  /* BEGIN: Compute A, B and Q for dt  and run prediction step computing x-, P-, z- */

  const double dt2 = 0.5*dT*dT;
  const double dt3 = dt2*dT;

  A << 1,  0,  0, dT,  0,  0,
       0,  1,  0,  0, dT,  0,
       0,  0,  1,  0,  0, dT,
       0,  0,  0,  1,  0,  0,
       0,  0,  0,  0,  1,  0,
       0,  0,  0,  0,  0,  1;

  B << dt2,   0,   0,
         0, dt2,   0,
         0,   0, dt2,
        dT,   0,   0,
         0,  dT,   0,
         0,   0,  dT;

  Q << dT,  0,  0,  1,  0,  0,
        0, dT,  0,  0,  1,  0,
        0,  0, dT,  0,  0,  1,
        1,  0,  0,  1,  0,  0,
        0,  1,  0,  0,  1,  0,
        0,  0,  1,  0,  0,  1;

  Q = sigma_sqr_process*dt2*Q;

  x_hat = A*x_hat + B*u;
  P_hat = A*P_hat*A.transpose() + Q;


  /* END: Compute A, B and Q for dt and end of running prediction step computing x-, P-, z-*/

  time_reference = time_reference_new;

	// Print all the debugging info you may need
  /*
  ROS_INFO ("\n\n");
  ROS_INFO ("Debugging prediction step (dt= %0.4fsec)", dT);
  ROS_INFO("Pos estimation %f %f %f", x_hat(0),x_hat(1),x_hat(2));
  ROS_INFO("Vel estimation %f %f %f", x_hat(3),x_hat(4),x_hat(5));
  */
}

void EstimatorNode::update_aruco()
{
    //Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);

    L = P_hat * C_aruco.transpose() * (C_aruco * P_hat * C_aruco.transpose() + R_aruco).inverse();
    x_hat = x_hat + L * (y_aruco-C_aruco*x_hat);
    P_hat = P_hat - L * C_aruco * P_hat;

    //P_hat = (I - L * C_aruco) * P_hat;
}

void EstimatorNode::update_twist()
{
    //Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);

    L = P_hat * C_twist.transpose() * (C_twist * P_hat * C_twist.transpose() + R_twist).inverse();
    x_hat = x_hat + L * (y_twist-C_twist*x_hat);
    P_hat = P_hat - L * C_twist * P_hat;
}

void EstimatorNode::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    ROS_INFO_ONCE("Estimator got first IMU message.");

    incomingImuMsg_ = *imu_msg;

    if (calibrating)
    {
        calib_imu_att_q_buffer.push_back(tf::Quaternion(imu_msg->orientation.x,imu_msg->orientation.y,imu_msg->orientation.z,imu_msg->orientation.w));
        calib_imu_ang_vel_buffer.push_back(tf::Vector3(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z));
        calib_imu_accel_buffer.push_back(tf::Vector3(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z));

        time_reference = ros::WallTime::now();
    }
    else
    {
        msgOdometry_.header.stamp = imu_msg->header.stamp;

        /* BEGIN: Process the acceleration: remove bias, rotate and remove gravity*/
        tf::Vector3     imu_accel ( imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
        tf::Quaternion  imu_att_q(imu_msg->orientation.x,imu_msg->orientation.y,imu_msg->orientation.z,imu_msg->orientation.w);

        imu_att_q = pose2imu_rotation*imu_att_q;
        //imu_accel = gravity_w + tf::quatRotate(imu_att_q, imu_accel-imu_accel_bias);
        imu_accel = gravity_w + tf::quatRotate(imu_att_q, imu_accel+imu_accel_bias);

        /* END: Process the acceleration: remove bias, rotate and remove gravity*/

        //u << imu_accel_bias[0],imu_accel_bias[1],imu_accel_bias[2];
        u << imu_accel[0],imu_accel[1],imu_accel[2];

        predict();

        publishPose();

    }
}

void EstimatorNode::GroundTruthPoseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    ROS_INFO_ONCE("Estimator got first ground truth pose message.");

    incomingGroundTruthPoseMsg_ = *pose_msg;
}

void EstimatorNode::TwistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist_msg)
{
    if(calibrating)
    {
      calib_twist_sensor_buffer.push_back(tf::Vector3( twist_msg->twist.twist.linear.x,
                                                       twist_msg->twist.twist.linear.y,
                                                       twist_msg->twist.twist.linear.z));
    }
    else
    {
        msgOdometry_.header.stamp = twist_msg->header.stamp;

        y_twist << twist_msg->twist.twist.linear.x, twist_msg->twist.twist.linear.y, twist_msg->twist.twist.linear.z;

        predict();

        update_twist();

        publishPose();
    }
}

void EstimatorNode::PoseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    ROS_INFO_ONCE("Estimator got first Aruco pose message.");

    incomingPoseMsg_ = *pose_msg;

    if (calibrating)
    {
        calib_pose_sensor_att_buffer.push_back(tf::Quaternion(pose_msg->pose.pose.orientation.x,
                                                              pose_msg->pose.pose.orientation.y,
                                                              pose_msg->pose.pose.orientation.z,
                                                              pose_msg->pose.pose.orientation.w));
        calib_pose_sensor_pos_buffer.push_back(tf::Vector3( pose_msg->pose.pose.position.x,
                                                            pose_msg->pose.pose.position.y,
                                                            pose_msg->pose.pose.position.z));
    }
    else
    {
        msgOdometry_.header.stamp = pose_msg->header.stamp;

        /* BEGIN: Generate the measurement y and call update*/
        y_aruco << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z;

        predict();

        update_aruco();

        /* END: Generate the measurement y and call update*/

        publishPose();
    }
}

void EstimatorNode::publishPose()
{
    //publish your data
    //ROS_INFO("Publishing ...");
    msgOdometry_.header.frame_id = "world";
    msgOdometry_.child_frame_id = "imu";

    // publish also as pose with covariance
    msgOdometry_.pose.pose.position.x = x_hat[0];
    msgOdometry_.pose.pose.position.y = x_hat[1];
    msgOdometry_.pose.pose.position.z = x_hat[2];

    msgOdometry_.twist.twist.linear.x = x_hat[3];
    msgOdometry_.twist.twist.linear.y = x_hat[4];
    msgOdometry_.twist.twist.linear.z = x_hat[5];

    // Take the orientation directly from IMU since we don't estimate it
    msgOdometry_.pose.pose.orientation = incomingImuMsg_.orientation;

    // fill in the values corresponding to position in the covariance

    for (int ii = 0; ii <3; ii++)
    {
        for (int jj = 0; jj<3; jj++)
        {
            msgOdometry_.pose.covariance[ii*6+jj] = P_hat(ii,jj);
            msgOdometry_.twist.covariance[ii*6+jj] = P_hat(ii+3,jj+3);
        }
    }

    odometry_pub_.publish(msgOdometry_);
}

void  EstimatorNode::readPars (ros::NodeHandle& nh) { 
    nh.getParam("estimation/aruco_covariance", sigma_sqr_aruco);
    nh.getParam("estimation/twist_covariance", sigma_sqr_twist);
    nh.getParam("estimation/process_covariance", sigma_sqr_process);
    nh.getParam("estimation/initial_error_covariance", sigma_sqr_P0);

    nh.getParam("estimation/initial_x", initial_pos[0]);
    nh.getParam("estimation/initial_y", initial_pos[1]);
    nh.getParam("estimation/initial_z", initial_pos[2]);
}

void  EstimatorNode::startCalibration () { 
  calibrating = true; 
  ROS_INFO_ONCE("Calibration initiated.");
}

void  EstimatorNode::endCalibration ()   { 

  imu_att_q_bias   = averageQuaternion(calib_imu_att_q_buffer);
  imu_ang_vel_bias = averageVector3(calib_imu_ang_vel_buffer);
  imu_accel_bias   = averageVector3(calib_imu_accel_buffer);

  if(calib_twist_sensor_buffer.empty())
      calib_twist_sensor_buffer.push_back(tf::Vector3(0.0, 0.0, 0.0));
  tf::Vector3 twist_sensor_offset = averageVector3(calib_twist_sensor_buffer);

  if(calib_pose_sensor_att_buffer.empty())
      calib_pose_sensor_att_buffer.push_back(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf::Quaternion pose_sensor_att_bias = averageQuaternion(calib_pose_sensor_att_buffer);

  if(calib_pose_sensor_pos_buffer.empty())
      calib_pose_sensor_pos_buffer.push_back(initial_pos);
  pose_sensor_pos_offset = averageVector3(calib_pose_sensor_pos_buffer);



  ROS_INFO("BEFORE IMU Accel. bias %f %f %f", imu_accel_bias.x(), imu_accel_bias.y(), imu_accel_bias.z());

  pose2imu_rotation = pose_sensor_att_bias*imu_att_q_bias.inverse();
  imu_accel_bias = -imu_accel_bias - tf::quatRotate( (pose2imu_rotation*imu_att_q_bias).inverse(), gravity_w);

  ROS_INFO_ONCE("Calibration ended. Summary: ");
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("IMU samples %d Pose samples %d", (int)calib_imu_att_q_buffer.size(), (int)calib_pose_sensor_pos_buffer.size());
  double roll, pitch, yaw;
  tf::Matrix3x3(imu_att_q_bias).getRPY(roll, pitch, yaw);
  ROS_INFO("IMU RPY bias %f %f %f", roll, pitch, yaw);
  ROS_INFO("IMU Ang.Vel bias %f %f %f", imu_ang_vel_bias.x(), imu_ang_vel_bias.y(), imu_ang_vel_bias.z());
  ROS_INFO("IMU Accel. bias %f %f %f", imu_accel_bias.x(), imu_accel_bias.y(), imu_accel_bias.z());
  ROS_INFO("Pose Sensor pose bias %f %f %f", pose_sensor_pos_offset.x(), pose_sensor_pos_offset.y(), pose_sensor_pos_offset.z());
  ROS_INFO("Twist Sensor twist bias %f %f %f", twist_sensor_offset.x(), twist_sensor_offset.y(), twist_sensor_offset.z());
  tf::Matrix3x3(pose_sensor_att_bias).getRPY(roll, pitch, yaw);
  ROS_INFO("Pose Sensor RPY bias %f %f %f", roll, pitch, yaw);
  tf::Matrix3x3(pose2imu_rotation).getRPY(roll, pitch, yaw);
  ROS_INFO("Offset Pose to IMU RPY %f %f %f", roll, pitch, yaw);
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 
  ROS_INFO("**********************************"); 

  // Set initial state values
#if 0
  x_hat << pose_sensor_pos_offset[0], pose_sensor_pos_offset[1], pose_sensor_pos_offset[2],
           twist_sensor_offset[0], twist_sensor_offset[1], twist_sensor_offset[2];
#endif
  x_hat << pose_sensor_pos_offset[0], pose_sensor_pos_offset[1], pose_sensor_pos_offset[2], 0, 0, 0;

  // free memory
  calib_imu_att_q_buffer.clear(); 
  calib_imu_ang_vel_buffer.clear(); 
  calib_imu_accel_buffer.clear(); 
  calib_pose_sensor_att_buffer.clear(); 
  calib_pose_sensor_pos_buffer.clear();
  calib_twist_sensor_buffer.clear();


  calibrating = false;
}

tf::Quaternion EstimatorNode::averageQuaternion(std::vector<tf::Quaternion> vec) // It is hacky to do it in RPY
{  
  if (vec.size() == 0) 
    return tf::Quaternion();

  double roll, pitch, yaw;
  double calib_roll, calib_pitch, calib_yaw;
  calib_roll = calib_pitch = calib_yaw = 0.0;
  for(int i = 0; i < vec.size(); i++) 
  {
    tf::Matrix3x3(vec[i]).getRPY(roll, pitch, yaw);
    calib_roll += roll;
    calib_pitch += pitch;
    calib_yaw += yaw;
  }
  calib_roll  = calib_roll / (double)vec.size();
  calib_pitch = calib_pitch / (double)vec.size();
  calib_yaw   = calib_yaw / (double)vec.size();

  return tf::createQuaternionFromRPY(calib_roll, calib_pitch, calib_yaw);
}

tf::Vector3 EstimatorNode::averageVector3(std::vector<tf::Vector3> vec)
{
  if (vec.size() == 0) 
    return tf::Vector3();

  tf::Vector3 res(0.0, 0.0, 0.0);
  for(int i = 0; i < vec.size(); i++) 
  {
    res += vec[i];
  }
  res /= vec.size();

  return res;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");
  ros::NodeHandle nh;
  ROS_INFO ("Starting estimator node"); 

  EstimatorNode estimator_node(nh);

  ROS_INFO("Waiting for simulation to start up...");  
  while (ros::WallTime::now().toSec() < 0.2)
  {
    ROS_INFO("Waiting for simulation to start up...");  
    ros::spinOnce();
    ros::Duration(0.01).sleep();     
  }

  //give time to start up the simulation
  ros::Duration(0.2).sleep();     

  // Initialize your filter / controller.
  ROS_INFO("Calibrating offsets for 2 secs...");
  estimator_node.startCalibration(); 
  // 2 secs for init the filters
  ros::WallTime time_reference = ros::WallTime::now();
  while ((ros::WallTime::now()-time_reference).toSec() < 2.0)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();     
  }

  estimator_node.endCalibration(); 

  ros::Publisher readyPub_ = nh.advertise<std_msgs::Empty>("filter_ready", 1, true);
  readyPub_.publish(std_msgs::Empty{});

  // let it go .. 
  ros::spin();

  return 0;
}
