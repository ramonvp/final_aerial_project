#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <map>

#include <voxblox_ros/esdf_server.h>

namespace voxblox
{
    class PathPlanner
    {

    public:
        PathPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        bool isCollisionFree(const Eigen::Vector3d &position);

    protected:
        
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        voxblox::EsdfServer esdf_server_;

        //parameters
        double p_collision_radius_;

    };
} // namespace voxblox
#endif