#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <voxblox_ros/esdf_server.h>
#include <final_aerial_project/voxblox_ompl_rrt.h>
#include <final_aerial_project/MakePlan.h>

namespace final_aerial_project
{

class PathPlanner
{
public:
    PathPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    bool isCollisionFree(const geometry_msgs::Point & point);

    bool makePlan(const geometry_msgs::PoseStamped & start_pose,
                  const geometry_msgs::PoseStamped & goal_pose,
                  trajectory_msgs::MultiDOFJointTrajectory & sampled_plan);

    double euclideanDistance(const geometry_msgs::Point & a, const geometry_msgs::Point & b) const;

    bool isPointInMap(const geometry_msgs::Point & point);

    void loop();

protected:
    bool makePlanService(final_aerial_project::MakePlanRequest &req,
                         final_aerial_project::MakePlanResponse &res);

    void computeMapBounds(Eigen::Vector3d* lower_bound,
                          Eigen::Vector3d* upper_bound) const;


    bool makePlan_RRT(const geometry_msgs::PoseStamped & start_pose,
                      const geometry_msgs::PoseStamped & goal_pose,
                      trajectory_msgs::MultiDOFJointTrajectory & rrt_plan);

    double getMapDistance(const Eigen::Vector3d& position) const;

    double getMapDistance(const geometry_msgs::Point & point) const;

    double computePathLength(const mav_msgs::EigenTrajectoryPointVector& path);

    double computePointCost(const geometry_msgs::Point & point,
                            const geometry_msgs::Pose & goal_pose,
                            const geometry_msgs::Pose & start_pose);

    void publishGridSearchPoints(const std::string & frame_id, const std::vector<geometry_msgs::Point> & points);

    void publishGridSearchArrow(const std::string & frame_id, const geometry_msgs::Point & start, const geometry_msgs::Point & end);

    visualization_msgs::Marker createMarkerForPath(mav_msgs::EigenTrajectoryPointVector& path,
                                                   const std::string& frame_id);

    bool isPointInsideSearchGrid(const geometry_msgs::Pose & start_pose, const geometry_msgs::Pose & goal_pose) const;

    bool isCollisionFree(const Eigen::Vector3d &position);


    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    voxblox::EsdfServer esdf_server_;
    voxblox::EsdfMap::Ptr esdf_map_;
    voxblox::TsdfMap::Ptr tsdf_map_;

    ros::ServiceServer plan_service_;
    ros::Publisher rviz_marker_pub_;
    ros::Publisher path_marker_pub_;

    //parameters
    double p_collision_radius_;
    bool try_rrt_;

    // search grid
    int Nx_;
    int Ny_;
    int Nz_;
    double grid_sep_;

    mav_planning::VoxbloxOmplRrt rrt_;

    double max_z_;
    double distance_gain_;
    double angle_gain_;
    double obstacle_gain_;

};

} // namespace final_aerial_project
#endif
