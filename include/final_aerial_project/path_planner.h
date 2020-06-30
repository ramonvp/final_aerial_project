#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <voxblox_ros/esdf_server.h>
#include <final_aerial_project/voxblox_ompl_rrt.h>
#include <final_aerial_project/MakePlan.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace final_aerial_project
{

struct PointCost
{
    double distance_cost{0.0};
    double angle_cost{0.0};
    double obstacle_cost{0.0};
    double known_cost{0.0};

    double cost() {
      return distance_cost + angle_cost + obstacle_cost + known_cost;
    }

    std::vector<double> costs() {
      return std::vector<double>{
        distance_cost,
        angle_cost,
        obstacle_cost,
        known_cost};
    }
};

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

    void publishGoalPointMarker(const geometry_msgs::PoseStamped & pose);

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

    PointCost computePointCost(const geometry_msgs::Point & point,
                            const geometry_msgs::Pose & goal_pose,
                            const geometry_msgs::Pose & start_pose);

    void publishGridSearchPoints(const std::string & frame_id, const std::vector<geometry_msgs::Point> & points, bool is_full);

    void publishGridSearchArrow(const std::string & frame_id, const geometry_msgs::Point & start, const geometry_msgs::Point & end);

    visualization_msgs::Marker createMarkerForPath(mav_msgs::EigenTrajectoryPointVector& path,
                                                   const std::string& frame_id);

    bool isPointInsideSearchGrid(const geometry_msgs::Pose & start_pose, const geometry_msgs::Pose & goal_pose) const;

    bool isCollisionFree(const Eigen::Vector3d &position);

    bool isPointObserved(const geometry_msgs::Point & point) const;
    bool isPointObserved(const Eigen::Vector3d &position) const;


    // generate a grid of points around <point> with optional yaw
    std::vector<geometry_msgs::Point> generateGrid(const geometry_msgs::Point & point, int nx, int ny, int nz, double yaw, double sep);

    // generate a grid of points around <point> with optional yaw
    std::vector<geometry_msgs::Point> generateGridTowardsGoal(const geometry_msgs::Point & start_point, const geometry_msgs::Point & end_point);

    void getInfoCallback(const geometry_msgs::Point::ConstPtr & msg);

    std_msgs::ColorRGBA colorFromIndex(int index) const;

    void createCylinderCost(const std::vector<double> & costs,
                            const geometry_msgs::Point & point,
                            const int start_id,
                            visualization_msgs::MarkerArray & marker_array) const;

    geometry_msgs::Pose pose_from_7D_vec(const std::vector<double> & poseVec);

    void createStaticPaths();

    // fill trajectory if start and goal poses match any of the
    // predefined paths. Return true if a match was found, false otherwise.
    bool isPredefinedPath(const geometry_msgs::Pose & start_pose,
                          const geometry_msgs::Pose & goal_pose,
                          trajectory_msgs::MultiDOFJointTrajectory & sampled_plan);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    voxblox::EsdfServer esdf_server_;
    voxblox::EsdfMap::Ptr esdf_map_;
    voxblox::TsdfMap::Ptr tsdf_map_;

    ros::ServiceServer plan_service_;
    ros::Publisher rviz_marker_pub_;
    ros::Publisher path_marker_pub_;
    ros::Publisher grid_cost_marker_pub_;

    ros::Subscriber infoSub_;

    //parameters
    double p_collision_radius_;
    bool try_rrt_;

    // search grid
    int Nx_;
    int Ny_;
    int Nz_;
    double grid_sep_;
    int overshoot_;

    mav_planning::VoxbloxOmplRrt rrt_;

    double max_z_;
    double distance_gain_;
    double angle_gain_;
    double obstacle_gain_;
    double known_gain_;


    bool use_cheating_paths_;
    trajectory_msgs::MultiDOFJointTrajectory fromChargeToShelve_;
    trajectory_msgs::MultiDOFJointTrajectory fromCheckpadToHusky_;

    // Pose of elements
    geometry_msgs::Pose husky_pose_;
    geometry_msgs::Pose shelve_pose_;

    // bases info to load from parameter server
    geometry_msgs::Point check_pad_position_;
    geometry_msgs::Point charging_pad_position_;

};

} // namespace final_aerial_project
#endif
