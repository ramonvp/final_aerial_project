#include <ros/ros.h>
#include "aerial_project/path_planner.h"

namespace voxblox
{

    PathPlanner::PathPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh),
          nh_private_(nh_private),
          esdf_server_(nh_, nh_private_)
    {

        nh_private_.param("collision_radius", p_collision_radius_, 1.0);
    }

    // Method that checks whether a point is occupied or not 
    // The point is collision free if it's distance to the nearest obstacle is bigger
    // than the collision radius defined
    bool PathPlanner::isCollisionFree(const Eigen::Vector3d &position)
    {
        double distance = 0.0;
        const bool kInterpolate = false;
        esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
            position, kInterpolate, &distance);

        if (distance < p_collision_radius_)
        {
            return false;
        }
        return true;
    }

} // namespace voxblox
