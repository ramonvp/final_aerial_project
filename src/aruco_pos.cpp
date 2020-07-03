#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

std::ostream& operator<< (std::ostream& os, const tf::StampedTransform & tf)
{
    tf::Quaternion q = tf.getRotation();
    os << "Translation (" << tf.getOrigin().x() << "," << tf.getOrigin().y() << "," << tf.getOrigin().z() << ")\n";
    os << "Rotation (" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ")";
    return os;
}

class ArucoLocalization
{
public:
    ArucoLocalization()
    {
        load_params();

        waitForTf();

        current_pose_.header.frame_id = global_frame_;

        load_markers_positions();

        // Init publishers
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_output", 10);

        // Init subscribers
        aruco_sub = nh_.subscribe("fiducial_transforms", 1, &ArucoLocalization::fiducial_callback, this);
    }

    void waitForTf()
    {
        tf::TransformListener tf_listener(ros::Duration(10.0));

        // we need to make sure that the transform between the base_link frame and
        // the ventral camera frame is available
        ros::Time last_error = ros::Time::now();
        std::string tf_error;
        while (ros::ok() &&
               !tf_listener.waitForTransform(base_link_frame_, camera_link_frame_, ros::Time(),
                                      ros::Duration(0.1), ros::Duration(0.01),
                                      &tf_error))
        {
            ros::spinOnce();
            if (last_error + ros::Duration(5.0) < ros::Time::now()) {
                ROS_WARN(
                    "Timed out waiting for transform from %s to %s to become available "
                    "before subscribing to costmap, tf error: %s",
                    camera_link_frame_.c_str(), base_link_frame_.c_str(), tf_error.c_str());
                last_error = ros::Time::now();
            }
            // The error string will accumulate and errors will typically be the same,
            // so the last will do for the warning above. Reset the string here to avoid
            // accumulation.
            tf_error.clear();
        }

        tf_listener.lookupTransform(base_link_frame_, camera_link_frame_, ros::Time(0), cam_to_base_link_tf_);
        ROS_DEBUG_STREAM("TF between base_link and camera is: " << cam_to_base_link_tf_);
    }

    void load_params()
    {
        ros::NodeHandle nh_private_("~");

        nh_private_.param<double>("weighting_scale", weighting_scale_, 1e9);
        nh_private_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        nh_private_.param<std::string>("camera_frame", camera_link_frame_, "camera_link");
        nh_private_.param<std::string>("global_frame", global_frame_, "world");
    }

    void load_markers_positions()
    {
        ros::NodeHandle nh_private_("~");

        int num_markers {-1};
        if( !nh_private_.getParam("/groundMarkers/numberOfMarkers", num_markers) )
        {
            ROS_WARN("Number of markers not provided, will assume from size of idList");
        }
        else if( num_markers <= 0)
        {
            ROS_FATAL("Invalid number of markers: %d", num_markers);
            exit(0);
        }


        XmlRpc::XmlRpcValue idList;
        bool ids_exists = nh_private_.getParam("/groundMarkers/idList", idList);
        if( !ids_exists)
        {
            ROS_FATAL("Missing /groundMarkers/idList!");
            exit(0);
        }

        ROS_ASSERT(idList.getType() == XmlRpc::XmlRpcValue::TypeArray);

        if( num_markers == -1)
        {
            num_markers = idList.size();
        }
        ROS_ASSERT(num_markers == idList.size());

        std::string format = "rpy";

        if( !nh_private_.getParam("/groundMarkers/orientationFormat", format) )
        {
            ROS_WARN("Missing /groundMarkers/orientationFormat, assuming format RPY");
        }

        int size_position = 3; // always contains X,Y,Z
        int size_orientation = 0;
        if(format == "rpy")
        {
            size_orientation = 3;
        }
        else if(format == "quat")
        {
            size_orientation = 4;
        }
        else
        {
            ROS_FATAL("Unrecognized quaternion format: %s", format.c_str());
            exit(0);
        }

        int size = size_position + size_orientation;

        XmlRpc::XmlRpcValue poseList;
        if( !nh_private_.getParam("/groundMarkers/poseList", poseList) )
        {
            ROS_FATAL("Missing /groundMarkers/poseList!");
            exit(0);
        }
        ROS_ASSERT(num_markers == poseList.size()/size);

        for( int i = 0; i < num_markers; i++)
        {
            int id = static_cast<int>(idList[i]);
            geometry_msgs::Pose pose;
            pose.position.x = static_cast<double>(poseList[size*i + 0]);
            pose.position.y = static_cast<double>(poseList[size*i + 1]);
            pose.position.z = static_cast<double>(poseList[size*i + 2]);

            double q1 = static_cast<double>(poseList[size*i + 3]);
            double q2 = static_cast<double>(poseList[size*i + 4]);
            double q3 = static_cast<double>(poseList[size*i + 5]);

            tf::Quaternion quat;
            if(format == "rpy")
            {
                quat = tf::createQuaternionFromRPY(q1, q2, q3);
            }
            else if( format == "quat")
            {
                double w = static_cast<double>(poseList[size*i + 6]);
                quat = tf::Quaternion(q1, q2, q3, w);
            }
            tf::quaternionTFToMsg(quat, pose.orientation);

            marker_positions_.insert( std::make_pair(id, pose) );
        }
    }

    void fiducial_callback(const fiducial_msgs::FiducialTransformArrayConstPtr& msg)
    {
        ROS_INFO_ONCE("[ArucoPos] First fiducials transform received");
        if( msg->transforms.size() == 0)
            return;

        for(const auto & t : msg->transforms)
        {
            auto it = marker_positions_.find(t.fiducial_id);
            if( it == marker_positions_.end() )
            {
                ROS_ERROR("Marker with ID=%d is not in the dictionary!", t.fiducial_id);
                continue;
            }

            const auto & marker_pose = it->second;
            ROS_DEBUG("Marker %d has pose: %f, %f, %f, yaw=%f",
                     t.fiducial_id,
                     marker_pose.position.x, marker_pose.position.y, marker_pose.position.z,
                     tf::getYaw(marker_pose.orientation));

            current_pose_.header.stamp = msg->header.stamp;

            tf::Transform aruco_tf;
            tf::transformMsgToTF(t.transform, aruco_tf);

            tf::Pose world_aruco_tf;
            tf::poseMsgToTF(marker_pose, world_aruco_tf);

            tf::StampedTransform base_link_tf;
            base_link_tf.setData( world_aruco_tf * aruco_tf.inverse() *  cam_to_base_link_tf_);

            ROS_DEBUG_STREAM("Final TF Base link: " << base_link_tf);

            geometry_msgs::TransformStamped base_link_tf_msg;
            tf::transformStampedTFToMsg(base_link_tf, base_link_tf_msg);

            current_pose_.pose.pose.position.x  = base_link_tf_msg.transform.translation.x;
            current_pose_.pose.pose.position.y  = base_link_tf_msg.transform.translation.y;
            current_pose_.pose.pose.position.z  = base_link_tf_msg.transform.translation.z;
            current_pose_.pose.pose.orientation = base_link_tf_msg.transform.rotation;

            //double pos_covar = weighting_scale_ / t.fiducial_area;//* t.object_error;
            double pos_covar = -0.036e-06 * t.fiducial_area + 0.4576e-03;
            if(pos_covar < 1.0e-06)
              pos_covar = 1.0e-06;
            double orient_covar = pos_covar;

            for (int i = 0; i < 3; i++){
                current_pose_.pose.covariance[6*i+i] = pos_covar;
                current_pose_.pose.covariance[6*(i+3)+i+3] = orient_covar;
            }

            pose_pub_.publish(current_pose_);
        }
    }

private:
    // contains the poses by id of all markers
    std::map<int, geometry_msgs::Pose> marker_positions_;

    ros::NodeHandle nh_;

    ros::Subscriber aruco_sub;
    ros::Publisher pose_pub_;

    geometry_msgs::PoseWithCovarianceStamped current_pose_;

    tf::StampedTransform cam_to_base_link_tf_;

    std::string base_link_frame_;
    std::string camera_link_frame_;
    std::string global_frame_;

    double weighting_scale_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "aruco_localization");

    ArucoLocalization aruco_localization;

    ros::spin();

    return 0;
}
