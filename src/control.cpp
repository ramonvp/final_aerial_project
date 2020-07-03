#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mutex>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <angles/angles.h>
#include <tf/tf.h>

#include <dynamic_reconfigure/server.h>
#include <final_aerial_project/ControllerConfig.h>
#include <final_aerial_project/PID.h>

std::ostream& operator<< (std::ostream& os, const tf::Vector3 & v)
{
    os << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
    return os;
}

// convenient access
enum {X,Y,Z,YAW};

std::mutex mutex_;

// Feedbacks
sensor_msgs::Imu latest_imu;
geometry_msgs::PoseWithCovarianceStamped	latest_pose;

nav_msgs::Path latest_trajectory;
int current_index {0};

// starting point
geometry_msgs::PoseStamped	initial_pose;

// Setpoints
tf::Vector3 setpoint_pos {0,0,0};
double setpoint_yaw {0};

tf::Vector3 error_pos {0,0,0};
double error_yaw {0};

tf::Vector3 integral_error {0,0,0};
double integral_error_yaw {0};

tf::Vector3 previous_error_pos {0,0,0};
double previous_error_yaw {0};

// Gravity 
double 	gravity_compensation = 0.0 ;
float 	gravity              = 9.54;  

// PID control gains and limits 
double  x_kp, x_ki, x_integral_limit, x_kd, 
			y_kp, y_ki, y_kd, y_integral_limit, 
			z_kp, z_ki, z_kd, z_integral_limit, 
			yaw_kp, yaw_ki, yaw_kd, yaw_integral_limit,  
			x_vel_limit, y_vel_limit, z_vel_limit, yaw_vel_limit;


// Velocity commands and limits
float x_raw_vel_cmd, y_raw_vel_cmd, z_raw_vel_cmd, yaw_raw_vel_cmd;
float maxXVel, maxYVel, maxZVel, maxYawVel;
float x_vel_cmd, y_vel_cmd, z_vel_cmd, yaw_vel_cmd;

// Acceleration feedback for feedforward 
tf::Vector3		body_accel;

// publisher to confirm current trajectory
ros::Publisher trajectory_pub;

// pid publisher for debugging and tuning
ros::Publisher pid_publisher[4];

double waypoint_accuracy {0.5};
double yaw_accuracy {0.02};
bool enable_rviz_goal {false};
bool debug_pid {false};

void reset_i_terms()
{
  integral_error[0] = 0.0;
  integral_error[1] = 0.0;
  integral_error[2] = 0.0;
  integral_error_yaw = 0.0;

  previous_error_pos[X] = 0.0;
  previous_error_pos[Y] = 0.0;
  previous_error_pos[Z] = 0.0;
  previous_error_yaw = 0.0;
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  ROS_INFO_ONCE("[CONTROLLER] First Imu msg received ");
	latest_imu = *msg; // Handle IMU data.
}
void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  ROS_INFO_ONCE("[CONTROLLER] First Pose msg received ");
	latest_pose = *msg; 	// Handle pose measurements.
}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
  ROS_INFO_ONCE("[CONTROLLER] First Odom msg received ");
  latest_pose.header = msg->header;
  latest_pose.pose = msg->pose; 	// Handle pose measurements.
}

void newSetpoint(const geometry_msgs::Pose & pose)
{
  setpoint_pos[0] = pose.position.x;
  setpoint_pos[1] = pose.position.y;
  setpoint_pos[2] = pose.position.z;
  setpoint_yaw    = tf::getYaw(pose.orientation);

  reset_i_terms();
}

void setTrajectoryFromPoint(const geometry_msgs::PoseStamped & wp)
{
    std::unique_lock<std::mutex> guard(mutex_);

    // Clear all pending waypoints.
    latest_trajectory.poses.clear();
    latest_trajectory.poses.push_back(wp);

    ROS_INFO ("WP 0\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t",
      wp.pose.position.x, wp.pose.position.y, wp.pose.position.z, tf::getYaw(wp.pose.orientation));

    current_index = 0;
    newSetpoint(wp.pose);
    trajectory_pub.publish(latest_trajectory);
}

void poseCommandCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  ROS_INFO_ONCE("[CONTROLLER] First Command Pose msg received ");
  setTrajectoryFromPoint(*msg);
}

void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ROS_INFO_ONCE("[CONTROLLER] First RViz Pose msg received ");
    geometry_msgs::PoseStamped newPose = *msg;
    // RViz always sends 2D goal with z = 0, so copy from current z
    newPose.pose.position.z = latest_pose.pose.pose.position.z;
    setTrajectoryFromPoint(newPose);
}

/* This function receives a trajectory of type MultiDOFJointTrajectoryConstPtr from the waypoint_publisher 
	and converts it to a Path in "latest_trajectory" to send it to rviz and use it to fly */ 
void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  std::unique_lock<std::mutex> guard(mutex_);

    // Clear all pending waypoints.
  latest_trajectory.poses.clear();

  // fill the header of the latest trajectory
  latest_trajectory.header = msg->header; 
  latest_trajectory.header.frame_id = "world" ;

  const size_t n_commands = msg->points.size();

  if(n_commands < 1)
  {
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  ROS_INFO("New trajectory with %d waypoints", (int) n_commands);

  // Extract the waypoints and print them
  for (size_t i = 0; i < n_commands; ++i) {

    geometry_msgs::PoseStamped wp; 
    wp.header.frame_id = msg->header.frame_id;
    wp.pose.position.x  = msg->points[i].transforms[0].translation.x;
    wp.pose.position.y  = msg->points[i].transforms[0].translation.y;
    wp.pose.position.z  = msg->points[i].transforms[0].translation.z;
    wp.pose.orientation = msg->points[i].transforms[0].rotation;
    
    latest_trajectory.poses.push_back(wp);

    ROS_INFO ("WP %d\t:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t", (int)i, 
    	wp.pose.position.x, wp.pose.position.y, wp.pose.position.z, tf::getYaw(wp.pose.orientation));
  }
  current_index = 0; 
  newSetpoint(latest_trajectory.poses[0].pose);

  trajectory_pub.publish(latest_trajectory);
}


/// Dynamic reconfigureCallback
void reconfigure_callback(final_aerial_project::ControllerConfig &config, uint32_t level)
{
	// Copy new configuration
	// m_config = config;

	gravity_compensation = config.gravity_compensation;

	x_kp = config.x_kp;
	x_ki = config.x_ki;
	x_kd = config.x_kd; 
	x_integral_limit = config.x_integral_limit;

	y_kp = config.y_kp;
	y_ki = config.y_ki; 
	y_kd = config.y_kd; 
	y_integral_limit = config.y_integral_limit;
	
	z_kp = config.z_kp;
	z_ki = config.z_ki; 
	z_kd = config.z_kd; 
	z_integral_limit = config.z_integral_limit;
	
	yaw_kp = config.yaw_kp; 
	yaw_ki = config.yaw_ki; 
	yaw_kd = config.yaw_kd; 
	yaw_integral_limit = config.yaw_integral_limit;

	x_vel_limit = config.x_vel_limit;
	y_vel_limit = config.y_vel_limit;
	z_vel_limit = config.z_vel_limit;
	yaw_vel_limit = config.yaw_vel_limit;

	maxXVel		= config.x_vel_limit;
	maxYVel		= config.y_vel_limit;
	maxZVel		= config.z_vel_limit;
	maxYawVel	= config.yaw_vel_limit;

	ROS_INFO (" ");
  ROS_INFO ("[CONTROLLER] Reconfigure callback have been called with new Settings ");
	
}

tf::Vector3 rotateZ (tf::Vector3 input_vector, float angle)
{
	tf::Quaternion quat;
	quat.setRPY(0.0, 0.0, angle);
	tf::Transform transform (quat);
	
	return (transform * input_vector);
}

double distanceToSetpoint()
{
    const double dx = setpoint_pos[0]-latest_pose.pose.pose.position.x;
    const double dy = setpoint_pos[1]-latest_pose.pose.pose.position.y;
    const double dz = setpoint_pos[2]-latest_pose.pose.pose.position.z;

    return sqrt(dx*dx + dy*dy + dz*dz);
}

bool setPointReached()
{
    double distance = distanceToSetpoint();
    error_yaw = angles::shortest_angular_distance(tf::getYaw(latest_pose.pose.pose.orientation), setpoint_yaw);
    return (distance < waypoint_accuracy) && (abs(error_yaw) < yaw_accuracy);
}

void read_parameters(ros::NodeHandle & nh_params)
{
  /*  Not really needed, since dynamic_reconfigure will provide the
      values stored in the controller.yaml file
    nh_params.param("gravity_compensation", gravity_compensation, 0.0);
    nh_params.param("x_kp", x_kp, 0.0);
    nh_params.param("x_ki", x_ki, 0.0);
    nh_params.param("x_kd", x_kd, 0.0);
    nh_params.param("x_integral_limit", x_integral_limit, 0.0);

    nh_params.param("y_kp", y_kp, 0.0);
    nh_params.param("y_ki", y_ki, 0.0);
    nh_params.param("y_kd", y_kd, 0.0);
    nh_params.param("y_integral_limit", y_integral_limit, 0.0);

    nh_params.param("z_kp", z_kp, 0.0);
    nh_params.param("z_ki", z_ki, 0.0);
    nh_params.param("z_kd", z_kd, 0.0);
    nh_params.param("z_integral_limit", z_integral_limit, 0.0);

    nh_params.param("yaw_kp", yaw_kp, 0.0);
    nh_params.param("yaw_ki", yaw_ki, 0.0);
    nh_params.param("yaw_kd", yaw_kd, 0.0);
    nh_params.param("yaw_integral_limit", yaw_integral_limit, 0.0);

    nh_params.param("x_vel_limit", x_vel_limit, 0.0);
    nh_params.param("y_vel_limit", y_vel_limit, 0.0);
    nh_params.param("z_vel_limit", z_vel_limit, 0.0);
    nh_params.param("yaw_vel_limit", yaw_vel_limit, 0.0);

    maxXVel = x_vel_limit;
    maxYVel = y_vel_limit;
    maxZVel = z_vel_limit;
    maxYawVel = yaw_vel_limit;
  */

  nh_params.param("enable_rviz_goal", enable_rviz_goal, enable_rviz_goal);
  nh_params.param("debug_pid", debug_pid, debug_pid);
  nh_params.param("waypoint_accuracy", waypoint_accuracy, waypoint_accuracy);
  nh_params.param("yaw_accuracy", yaw_accuracy, yaw_accuracy);

  // Read initial setpoint from parameter server
  nh_params.param("start_pose_x", initial_pose.pose.position.x, 0.0);
  nh_params.param("start_pose_y", initial_pose.pose.position.y, 0.0);
  nh_params.param("start_pose_z", initial_pose.pose.position.z, 1.0);
  double initial_yaw;
  nh_params.param("start_yaw", initial_yaw, 0.0);
  initial_pose.pose.orientation = tf::createQuaternionMsgFromYaw(initial_yaw);
  initial_pose.header.frame_id = "world";
  initial_pose.header.stamp = ros::Time::now();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	ros::NodeHandle nh_params("~");
	
  ROS_INFO("[CONTROLLER] Running controller");

  read_parameters(nh_params);
	
	// Inputs: imu and pose messages, and the desired trajectory 
  ros::Subscriber imu_sub   = nh.subscribe("imu",  1, &imuCallback);
  ros::Subscriber odom_sub  = nh.subscribe("odom", 1, &odomCallback);
  ros::Subscriber cmd_sub   = nh.subscribe("command/pose", 1, &poseCommandCallback);
	ros::Subscriber traj_sub  = nh.subscribe("command/trajectory", 1, &MultiDofJointTrajectoryCallback); 

  // Enable the RViz button 2D Nav Goal for easy testing
  ros::Subscriber rviz_goal_sub;
  if(enable_rviz_goal)
  {
      rviz_goal_sub = nh.subscribe("/move_base_simple/goal", 1, &rvizGoalCallback);
  }

	// Outputs: some platforms want linear velocity (ardrone), others rollpitchyawratethrust (firefly)
	// and the current trajectory 
	ros::Publisher rpyrt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1);
	ros::Publisher vel_command_pub   = nh.advertise<geometry_msgs::Twist>("command/velocity", 1);
  trajectory_pub = nh.advertise<nav_msgs::Path>("current_trajectory", 1, true);

  if(debug_pid)
  {
      pid_publisher[X] = nh.advertise<final_aerial_project::PID>("pid/x", 1);
      pid_publisher[Y] = nh.advertise<final_aerial_project::PID>("pid/y", 1);
      pid_publisher[Z] = nh.advertise<final_aerial_project::PID>("pid/z", 1);
      pid_publisher[YAW] = nh.advertise<final_aerial_project::PID>("pid/yaw", 1);
  }

	// Start the dynamic_reconfigure server
  dynamic_reconfigure::Server<final_aerial_project::ControllerConfig> server;
  dynamic_reconfigure::Server<final_aerial_project::ControllerConfig>::CallbackType f;
  f = boost::bind(&reconfigure_callback, _1, _2);
  server.setCallback(f);

	// Run the control loop and Fly to x=0m y=0m z=1m
  ROS_INFO_STREAM("[CONTROLLER] Going to starting position " << setpoint_pos);

  // set initial setpoint
  setTrajectoryFromPoint(initial_pose);

  //Elapsed time between pose messages
  ros::Time latest_pose_update_time = ros::Time::now();
  //Period for the control loop
  double control_loop_period = 0.01; // 10 ms => 100Hz
  ros::Rate rate(1.0/control_loop_period);

	while(ros::ok())
	{
		ros::spinOnce();

    std::unique_lock<std::mutex> guard(mutex_);
		
    double delta_time_pose = (latest_pose.header.stamp - latest_pose_update_time).toSec() ;

		// Check if pose/imu/state data was received
    if (	(latest_pose.header.stamp.nsec > 0.0) &&
       (  (latest_pose.header.stamp - latest_pose_update_time).toSec() > 0.0) )
		{				
        latest_pose_update_time = latest_pose.header.stamp;
        if (setPointReached())
        {
            if (current_index < latest_trajectory.poses.size() - 1)
            {
                ROS_INFO("[CONTROLLER] Waypoint %d achieved! Moving to next waypoint", current_index);
                current_index++;
                newSetpoint(latest_trajectory.poses[current_index].pose);
            }
            else if(current_index == latest_trajectory.poses.size() - 1) // print once waypoint achieved
            {
                ROS_INFO("[CONTROLLER] Waypoint %d achieved! No more waypoints. Hovering", current_index);
                current_index++;
            }
        }
				
			/* BEGIN: Run your position loop 
				- compute your error in position and yaw
				- run the update of the PID loop to obtain the desired velocities
				 */
      error_pos[0] = setpoint_pos[0]-latest_pose.pose.pose.position.x;
      error_pos[1] = setpoint_pos[1]-latest_pose.pose.pose.position.y;
      error_pos[2] = setpoint_pos[2]-latest_pose.pose.pose.position.z;
      error_yaw = angles::shortest_angular_distance(tf::getYaw(latest_pose.pose.pose.orientation), setpoint_yaw);

      integral_error[0]  += error_pos[0]*delta_time_pose;
      integral_error[1]  += error_pos[1]*delta_time_pose;
      integral_error[2]  += error_pos[2]*delta_time_pose;
      integral_error_yaw += error_yaw*delta_time_pose;

      if( fabs(integral_error[0]) > x_integral_limit )
          integral_error[0] = std::copysign(x_integral_limit, integral_error[0]);
      if( fabs(integral_error[1]) > y_integral_limit )
          integral_error[1] = std::copysign(y_integral_limit, integral_error[1]);
      if( fabs(integral_error[2]) > z_integral_limit )
          integral_error[2] = std::copysign(z_integral_limit, integral_error[2]);
      if( fabs(integral_error_yaw) > yaw_integral_limit )
          integral_error_yaw = std::copysign(yaw_integral_limit, integral_error_yaw);


      tf::Vector3	diff_error;
      diff_error[X] = (error_pos[X] - previous_error_pos[X])/delta_time_pose;
      diff_error[Y] = (error_pos[Y] - previous_error_pos[Y])/delta_time_pose;
      diff_error[Z] = (error_pos[Z] - previous_error_pos[Z])/delta_time_pose;
      double diff_error_yaw = (error_yaw - previous_error_yaw)/delta_time_pose;

      tf::Vector3 command_pos;
      command_pos[0] = x_kp * error_pos[0] + x_ki * integral_error[0] + x_kd * diff_error[X];
      command_pos[1] = y_kp * error_pos[1] + y_ki * integral_error[1] + y_kd * diff_error[Y];
      command_pos[2] = z_kp * error_pos[2] + z_ki * integral_error[2] + z_kd * diff_error[Z];
      double command_yaw = yaw_kp * error_yaw + yaw_ki * integral_error_yaw + yaw_kd * diff_error_yaw;

      // store current error for next cycle
      previous_error_pos = error_pos;
      previous_error_yaw = error_yaw;

			// rotate velocities to align them with the body frame
			// convert from local to body coordinates (ignore Z)
			tf::Vector3 vector3 (command_pos[0] , command_pos[1] , 0.0);        
			vector3 = rotateZ (vector3, -tf::getYaw(latest_pose.pose.pose.orientation));        

			// your desired velocities should be stored in 
      x_raw_vel_cmd = vector3[X];
      y_raw_vel_cmd = vector3[Y];
      z_raw_vel_cmd = command_pos[Z];
			yaw_raw_vel_cmd = command_yaw; 
		  
			//Saturate  the velocities 			
			x_vel_cmd  = (x_raw_vel_cmd > maxXVel)   ? maxXVel  : ((x_raw_vel_cmd < -maxXVel)  ? -maxXVel  : x_raw_vel_cmd);
			y_vel_cmd  = (y_raw_vel_cmd > maxYVel)   ? maxYVel  : ((y_raw_vel_cmd < -maxYVel)  ? -maxYVel  : y_raw_vel_cmd);
			z_vel_cmd  = (z_raw_vel_cmd > maxZVel)   ? maxZVel  : ((z_raw_vel_cmd < -maxZVel)  ? -maxZVel  : z_raw_vel_cmd);
			yaw_vel_cmd  = (yaw_raw_vel_cmd > maxYawVel)   ? maxYawVel  : ((yaw_raw_vel_cmd < -maxYawVel)  ? -maxYawVel  : yaw_raw_vel_cmd);

      if(debug_pid)
      {
        // pid feedback
        final_aerial_project::PID pid[4];

        pid[X].setpoint = setpoint_pos[X];
        pid[X].input = latest_pose.pose.pose.position.x;
        pid[X].error = error_pos[X];
        pid[X].integral_error = integral_error[X];
        pid[X].diff_error = diff_error[X];
        pid[X].windup_limit = x_integral_limit;
        pid[X].output_p = x_kp * error_pos[X];
        pid[X].output_i = x_ki * integral_error[X];
        pid[X].output_d = x_kd * diff_error[X];

        pid[Y].setpoint = setpoint_pos[Y];
        pid[Y].input = latest_pose.pose.pose.position.y;
        pid[Y].error = error_pos[Y];
        pid[Y].integral_error = integral_error[Y];
        pid[Y].diff_error = diff_error[Y];
        pid[Y].windup_limit = y_integral_limit;
        pid[Y].output_p = y_kp * error_pos[Y];
        pid[Y].output_i = y_ki * integral_error[Y];
        pid[Y].output_d = y_kd * diff_error[Y];

        pid[Z].setpoint = setpoint_pos[Z];
        pid[Z].input = latest_pose.pose.pose.position.z;
        pid[Z].error = error_pos[Z];
        pid[Z].integral_error = integral_error[Z];
        pid[Z].diff_error = diff_error[Z];
        pid[Z].windup_limit = z_integral_limit;
        pid[Z].output_p = z_kp * error_pos[Z];
        pid[Z].output_i = z_ki * integral_error[Z];
        pid[Z].output_d = z_kd * diff_error[Z];

        pid[YAW].setpoint = setpoint_yaw;
        pid[YAW].input = tf::getYaw(latest_pose.pose.pose.orientation);
        pid[YAW].error = error_yaw;
        pid[YAW].integral_error = integral_error_yaw;
        pid[YAW].diff_error = diff_error_yaw;
        pid[YAW].windup_limit = yaw_integral_limit;
        pid[YAW].output_p = z_kp * error_yaw;
        pid[YAW].output_i = z_ki * integral_error_yaw;
        pid[YAW].output_d = z_kd * diff_error_yaw;

        pid[X].output = x_vel_cmd;
        pid[Y].output = y_vel_cmd;
        pid[Z].output = z_vel_cmd;
        pid[YAW].output = yaw_vel_cmd;

        pid_publisher[X].publish(pid[X]);
        pid_publisher[Y].publish(pid[Y]);
        pid_publisher[Z].publish(pid[Z]);
        pid_publisher[YAW].publish(pid[YAW]);
      }

			/* A) 
			 * Some platforms receive linear velocities in body frame. lets publish it as twist
			 */
			geometry_msgs::Twist velocity_cmd;

			velocity_cmd.linear.x = x_vel_cmd;
			velocity_cmd.linear.y = y_vel_cmd;
			velocity_cmd.linear.z = z_vel_cmd;

			velocity_cmd.angular.x = 0.03; // this is a hack for Ardrone, it ignores 0 vel messages, but setting it in these fields that are ignored helps to set a real 0 vel cmd
			velocity_cmd.angular.y = 0.05;
			velocity_cmd.angular.z = yaw_vel_cmd;

			vel_command_pub.publish(velocity_cmd);			

			/* B) 
			 * Some platforms receive instead Roll, Pitch YawRate and Thrust. We need to compute them. 
			 */

			// Map velocities in x and y directly to roll, pitch  is equivalent to a P controller with Kp=1
			// but you still need to compute thrust yourselves 
			
			
			//Extract vertical acceleration to compensate for gravity
			tf::Quaternion orientation;
			quaternionMsgToTF(latest_pose.pose.pose.orientation, orientation);
			tf::Transform imu_tf = tf::Transform(orientation, tf::Vector3(0,0,0));
			tf::Vector3 imu_accel(latest_imu.linear_acceleration.x,
                            latest_imu.linear_acceleration.y,
                            latest_imu.linear_acceleration.z);
			body_accel = imu_tf*imu_accel;
			
      float thrust =  z_vel_cmd + gravity_compensation - (body_accel[Z]-gravity);


			// Send to the attitude controller:
			// roll angle [rad], pitch angle  [rad], thrust [N][rad/s]           
			mav_msgs::RollPitchYawrateThrust msg;

			msg.header 	  = latest_pose.header; // use the latest information you have.
			msg.pitch 	  = x_vel_cmd;
			msg.roll 	  = -y_vel_cmd;		
			msg.thrust.z  = thrust;
			msg.yaw_rate  = yaw_vel_cmd;
			rpyrt_command_pub.publish(msg);

		}
	
    rate.sleep();
	}
	return 0;
}


