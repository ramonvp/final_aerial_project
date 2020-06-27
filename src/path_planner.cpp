#include <ros/ros.h>
#include "final_aerial_project/path_planner.h"
#include <visualization_msgs/Marker.h>
#include <voxblox/utils/planning_utils.h>
#include <angles/angles.h>

namespace final_aerial_project
{

PathPlanner::PathPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      esdf_server_(nh_, nh_private_),
      rrt_(nh_, nh_private_)
{

    nh_private_.param("collision_radius", p_collision_radius_, 1.0);

    nh_private_.param("grid_x", Nx_, 5);
    nh_private_.param("grid_y", Ny_, 5);
    nh_private_.param("grid_z", Nz_, 3);
    nh_private_.param("grid_separation", grid_sep_, 1.0);

    nh_private_.param("try_rrt", try_rrt_, true);

    nh_private_.param("max_z", max_z_, 1.5);

    nh_private_.param("distance_gain", distance_gain_, 1.0);
    nh_private_.param("angle_gain", angle_gain_, 1.0);
    nh_private_.param("obstacle_gain", obstacle_gain_, 1.0);

    plan_service_ = nh_private_.advertiseService("makePlan", &PathPlanner::makePlanService, this);
    rviz_marker_pub_ = nh_private_.advertise<visualization_msgs::Marker>("grid_search", 5);
    path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);

    esdf_map_ = esdf_server_.getEsdfMapPtr();
    CHECK(esdf_map_);
    tsdf_map_ = esdf_server_.getTsdfMapPtr();
    CHECK(tsdf_map_);

    rrt_.setRobotRadius(p_collision_radius_);
    rrt_.setOptimistic(false);

    rrt_.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());
    rrt_.setEsdfLayer(esdf_map_->getEsdfLayerPtr());

    // not sure about this, copied from rrt planner
    esdf_server_.setTraversabilityRadius(static_cast<float>(p_collision_radius_));
    esdf_server_.publishTraversable();
}

double PathPlanner::getMapDistance(const Eigen::Vector3d& position) const
{
    if (!esdf_map_)
    {
        ROS_ERROR("Invalid ESDF map, cannot get map distance");
        return 0.0;
    }

    double distance = 0.0;
    const bool kInterpolate = false;

    // Returns true if the point exists in the map AND is observed.
    bool point_known = esdf_map_->getDistanceAtPosition(position, kInterpolate, &distance);
    if (!point_known)
    {
        ROS_WARN("Point (%f,%f,%f) is unknown to the map",
                 position[0], position[1], position[2]);
        return 0.0;
    }

    return distance;
}

double PathPlanner::getMapDistance(const geometry_msgs::Point & point) const
{
    return getMapDistance(Eigen::Vector3d(point.x, point.y, point.z));
}

// Method that checks whether a point is occupied or not
// The point is collision free if it's distance to the nearest obstacle is bigger
// than the collision radius defined
bool PathPlanner::isCollisionFree(const Eigen::Vector3d &position)
{
    double d = getMapDistance(position);

    if( d <= p_collision_radius_)
    {
        ROS_ERROR("Point (%f,%f,%f) collides d = %f",
                  position[0],position[1],position[2], d);
    }

    return d > p_collision_radius_;
}

bool PathPlanner::isCollisionFree(const geometry_msgs::Point & point)
{
    return isCollisionFree(Eigen::Vector3d(point.x, point.y, point.z));
}

bool PathPlanner::isPointInsideSearchGrid(
    const geometry_msgs::Pose & start_pose,
    const geometry_msgs::Pose & goal_pose) const
{

  geometry_msgs::Point lower_bound;
  geometry_msgs::Point upper_bound;

  int i = 0;
  int j = 0;
  int k = 0;

  lower_bound.x = start_pose.position.x + (i - Nx_/2)*grid_sep_;
  lower_bound.y = start_pose.position.y + j*grid_sep_;
  lower_bound.z = start_pose.position.z + (k - Nz_/2)*grid_sep_;

  i = Nx_ - 1;
  j = Ny_ - 1;
  k = Nz_ - 1;
  upper_bound.x = start_pose.position.x + (i - Nx_/2)*grid_sep_;
  upper_bound.y = start_pose.position.y + j*grid_sep_;
  upper_bound.z = start_pose.position.z + (k - Nz_/2)*grid_sep_;

  return goal_pose.position.x >= lower_bound.x &&
         goal_pose.position.y >= lower_bound.y &&
         goal_pose.position.z >= lower_bound.z &&
         goal_pose.position.x <= upper_bound.x &&
         goal_pose.position.y <= upper_bound.y &&
         goal_pose.position.z <= upper_bound.z;
}

bool PathPlanner::isPointInMap(const geometry_msgs::Point & point)
{
    Eigen::Vector3d lower_bound(Eigen::Vector3d::Zero());
    Eigen::Vector3d upper_bound(Eigen::Vector3d::Zero());
    computeMapBounds(&lower_bound, &upper_bound);

    return point.x > lower_bound[0] && point.x < upper_bound[0] &&
           point.y > lower_bound[1] && point.y < upper_bound[1] &&
           point.z > lower_bound[2] && point.z < upper_bound[2];
}


visualization_msgs::Marker PathPlanner::createMarkerForPath(
    mav_msgs::EigenTrajectoryPointVector& path,
    const std::string& frame_id)
{
    visualization_msgs::Marker path_marker;

    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.a = 1.0;

    const int kMaxSamples = 1000;
    const int num_samples = path.size();
    int subsample = 1;
    while (num_samples / subsample > kMaxSamples) {
      subsample *= 10;
    }
    const double kMaxMagnitude = 1.0e4;

    path_marker.header.frame_id = frame_id;

    path_marker.header.stamp = ros::Time::now();
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.color = color;
    path_marker.color.a = 0.75;
    path_marker.ns = "poly";
    path_marker.scale.x = 0.075;
    path_marker.scale.y = path_marker.scale.x;
    path_marker.scale.z = path_marker.scale.x;

    path_marker.points.reserve(path.size() / subsample);
    int i = 0;
    for (const mav_msgs::EigenTrajectoryPoint& point : path) {
      i++;
      if (i % subsample != 0) {
        continue;
      }
      // Check that we're in some reasonable bounds.
      // Makes rviz stop crashing.
      if (point.position_W.maxCoeff() > kMaxMagnitude ||
          point.position_W.minCoeff() < -kMaxMagnitude) {
        continue;
      }

      geometry_msgs::Point point_msg;
      mav_msgs::pointEigenToMsg(point.position_W, &point_msg);
      path_marker.points.push_back(point_msg);
    }

    return path_marker;
}

bool PathPlanner::makePlan(const geometry_msgs::PoseStamped & start_pose,
                           const geometry_msgs::PoseStamped & goal_pose,
                           trajectory_msgs::MultiDOFJointTrajectory & sampled_plan)
{
    ROS_INFO("[PathPlanner] Make plan with start (%f,%f,%f), goal (%f,%f,%f)",
             start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z,
             goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);

    if( !isPointInMap(start_pose.pose.position) )
    {
        ROS_ERROR("Start Pose is not in map, cannot make a plan");
        return false;
    }

    sampled_plan.header.frame_id = goal_pose.header.frame_id;
    sampled_plan.header.stamp = ros::Time::now();

    if( try_rrt_ )
    {
        bool goal_in_map = isPointInMap(goal_pose.pose.position);
        if( goal_in_map )
        {
            // ask the RRT to look for a path
            bool rrt_plan_ok = makePlan_RRT(start_pose, goal_pose, sampled_plan);
            if(!rrt_plan_ok)
            {
                ROS_ERROR("RRT failed to produce a valid path, backfall to local steps");
            }
            else  // we're done
            {
                return true;
            }
        }
        else
        {
            ROS_WARN("RRT can't make plan because goal Pose is not in map, will calculate local steps");
        }
    }
    else
    {
        ROS_WARN("RRT is disabled by option try_rrt");
    }

    // Step 1: Generate a grid of possible goal points
    // close to the MAV that are not occupied
    double yaw = tf::getYaw(start_pose.pose.orientation) - M_PI/2;
    std::vector<geometry_msgs::Point> grid_search;
    for(int k = 0; k < Nz_; k++)
    {
        double offset_z = (k - Nz_/2)*grid_sep_;

        if( start_pose.pose.position.z + offset_z > max_z_ )
          continue;

        for(int j = 0; j < Ny_; j++)
        {
            double offset_y = (j - Ny_/2)*grid_sep_;
            for(int i = 0; i < Nx_; i++)
            {
                geometry_msgs::Point p;
                double offset_x = (i - Nx_/2)*grid_sep_;
                p.x = start_pose.pose.position.x + offset_x*cos(yaw) - offset_y*sin(yaw);
                p.y = start_pose.pose.position.y + offset_x*sin(yaw) + offset_y*cos(yaw);
                p.z = start_pose.pose.position.z + offset_z;

                if( isCollisionFree(p) )
                {
                    grid_search.push_back(p);
                    ROS_DEBUG("Accepted free point is: %f, %f, %f",
                             p.x, p.y, p.z);
                }
                else
                {
                  ROS_DEBUG("Discarded occupied point is: %f, %f, %f",
                           p.x, p.y, p.z);
                }
            }
        }
    }

     if(isPointInsideSearchGrid(start_pose.pose, goal_pose.pose) &&
        isCollisionFree(goal_pose.pose.position))
     {
        grid_search.push_back(goal_pose.pose.position);
        ROS_INFO("Goal point is in reach, accepted in search grid!");
     }

     ROS_INFO("Number of candidates in grid search: %lu", grid_search.size());

    // Step 2: for each possible location compute the cost
    double min_cost = 99999999;
    int min_cost_pos = -1;
    int i = 0;
    for( auto & p : grid_search)
    {
        double cost = computePointCost(p, goal_pose.pose, start_pose.pose);
        if( cost < min_cost )
        {
            min_cost = cost;
            min_cost_pos = i;
        }
        i++;
    }

    if( min_cost_pos != -1)
    {
        const geometry_msgs::Point & sp = grid_search[min_cost_pos];
        ROS_INFO("Selected next point is: %f, %f, %f",  sp.x, sp.y, sp.z);

        sampled_plan.header.frame_id = goal_pose.header.frame_id;
        sampled_plan.header.stamp = ros::Time::now();

        // Specify difference in direction
        double yaw = atan2( sp.y - start_pose.pose.position.y,
                            sp.x - start_pose.pose.position.x);
        tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, yaw);

        // initial point with correct heading
        trajectory_msgs::MultiDOFJointTrajectoryPoint initial_point;
        geometry_msgs::Transform t_initial;
        t_initial.translation.x = start_pose.pose.position.x;
        t_initial.translation.y = start_pose.pose.position.y;
        t_initial.translation.z = start_pose.pose.position.z;
        tf::quaternionTFToMsg(quat, t_initial.rotation);
        initial_point.transforms.push_back(t_initial);
        sampled_plan.points.push_back(initial_point);

        trajectory_msgs::MultiDOFJointTrajectoryPoint traj_point;
        geometry_msgs::Transform t;
        t.translation.x = sp.x;
        t.translation.y = sp.y;
        t.translation.z = sp.z;
        t.rotation.w = 1.0;

        tf::quaternionTFToMsg(quat, t.rotation);

        traj_point.transforms.push_back(t);
        sampled_plan.points.push_back(traj_point);

        publishGridSearchPoints(sampled_plan.header.frame_id, grid_search);
        publishGridSearchArrow(sampled_plan.header.frame_id, start_pose.pose.position, sp);

        return true;
    }
    else
    {
        ROS_ERROR("All candidates in grid search were discarded!");
    }

    return false;
}


bool PathPlanner::makePlanService(final_aerial_project::MakePlanRequest &req,
                                  final_aerial_project::MakePlanResponse &res)
{
    return makePlan(req.start_pose, req.goal_pose, res.sampled_plan);
}

void PathPlanner::publishGridSearchArrow(const std::string & frame_id, const geometry_msgs::Point & start, const geometry_msgs::Point & end)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.ns = "next_wp";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    //marker.pose = req.start_pose.pose;
    marker.pose.orientation.w = 1.0;

    // Scale is the diameter of the shape
    marker.scale.x = 0.03; // shaft diameter
    marker.scale.y = 0.06; // head diameter
    marker.scale.z = 0.06; // if not zero, head length

    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.points.push_back(start);
    marker.points.push_back(end);


    rviz_marker_pub_.publish(marker);
}

void PathPlanner::publishGridSearchPoints(const std::string & frame_id, const std::vector<geometry_msgs::Point> & points)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.ns = "grid_search";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    //marker.pose = req.start_pose.pose;
    marker.pose.orientation.w = 1.0;

    // Scale is the diameter of the shape
    marker.scale.x = 0.05;
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;

    marker.color.r = 1.0;
    marker.color.a = 0.6;
    marker.points = points;


    rviz_marker_pub_.publish(marker);
}

double PathPlanner::computePointCost(const geometry_msgs::Point & point,
                                     const geometry_msgs::Pose & goal_pose,
                                     const geometry_msgs::Pose & start_pose)
{
    double cost {0.0};

    double distance_to_goal = euclideanDistance(point,  goal_pose.position);

    double currentYaw = tf::getYaw(start_pose.orientation);
    double targetYaw = atan2(point.y - start_pose.position.y,
                             point.x - start_pose.position.x);
    double error_yaw = abs(angles::shortest_angular_distance(currentYaw, targetYaw));
    // normalize error angle, such that 180º means +1 in cost, error is [0, 1.0]
    error_yaw = error_yaw/M_PI;

    double obstacle_distance = getMapDistance(point);


    cost = distance_gain_ * distance_to_goal +
           angle_gain_    * error_yaw +
           obstacle_gain_ * (p_collision_radius_/obstacle_distance);

    return cost;
}

double PathPlanner::euclideanDistance(const geometry_msgs::Point & a, const geometry_msgs::Point & b) const
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

void PathPlanner::computeMapBounds(Eigen::Vector3d* lower_bound,
                                   Eigen::Vector3d* upper_bound) const
{
    if (esdf_map_) {
        voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(),
                                                  lower_bound, upper_bound);
    } else if (tsdf_map_) {
        voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                                  lower_bound, upper_bound);
    }
}


bool PathPlanner::makePlan_RRT(const geometry_msgs::PoseStamped & start,
                               const geometry_msgs::PoseStamped & goal,
                               trajectory_msgs::MultiDOFJointTrajectory & rrt_plan)
{
    mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

    mav_msgs::eigenTrajectoryPointFromPoseMsg(start, &start_pose);
    mav_msgs::eigenTrajectoryPointFromPoseMsg(goal, &goal_pose);

    // Setup latest copy of map.
    if (!(esdf_map_ &&
          esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) &&
        !(tsdf_map_ &&
          tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0)) {
      ROS_ERROR("Both maps are empty!");
      return false;
    }

    // Figure out map bounds!
    Eigen::Vector3d lower_bound_(Eigen::Vector3d::Zero());
    Eigen::Vector3d upper_bound_(Eigen::Vector3d::Zero());
    computeMapBounds(&lower_bound_, &upper_bound_);

    ROS_INFO_STREAM("Map bounds: " << lower_bound_.transpose() << " to "
                                   << upper_bound_.transpose() << " size: "
                                   << (upper_bound_ - lower_bound_).transpose());

    // Limit Z to the same as goal, to avoid jumping over the walls
    lower_bound_[2] = max_z_;
    upper_bound_[2] = max_z_;

    // Inflate the bounds a bit.
    constexpr double kBoundInflationMeters = 0.5;
    // Don't in flate in z. ;)
    rrt_.setBounds(lower_bound_ - Eigen::Vector3d(kBoundInflationMeters,
                                                  kBoundInflationMeters, kBoundInflationMeters),
                   upper_bound_ + Eigen::Vector3d(kBoundInflationMeters,
                                                  kBoundInflationMeters, kBoundInflationMeters));
    rrt_.setupProblem();

    ROS_INFO("Planning path.");

    if (getMapDistance(start_pose.position_W) < p_collision_radius_) {
        ROS_ERROR("Start pose occupied!");
        return false;
    }

    if (getMapDistance(goal_pose.position_W) < p_collision_radius_) {
        ROS_ERROR("Goal pose occupied!");
        return false;
    }

    mav_msgs::EigenTrajectoryPoint::Vector waypoints;
    //mav_trajectory_generation::timing::Timer rrtstar_timer("plan/rrt_star");
    bool success = rrt_.getPathBetweenWaypoints(start_pose, goal_pose, &waypoints);
    //rrtstar_timer.Stop();
    double path_length = computePathLength(waypoints);
    std::size_t num_vertices = waypoints.size();
    ROS_INFO("RRT* Success? %d Path length: %f Vertices: %lu", success, path_length, num_vertices);

    if (!success)
    {
        return false;
    }

    //trajectory_msgs::MultiDOFJointTrajectory
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(waypoints, "base_link", &rrt_plan);

    // send the trajectory to RViz
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(
          createMarkerForPath(waypoints, rrt_plan.header.frame_id));
    path_marker_pub_.publish(marker_array);

    return true;
}

// Computes total path length of a sampled path.
double PathPlanner::computePathLength(const mav_msgs::EigenTrajectoryPointVector& path)
{
    Eigen::Vector3d last_point;
    double distance = 0;
    for (size_t i = 0; i < path.size(); ++i) {
      const mav_msgs::EigenTrajectoryPoint& point = path[i];

      if (i > 0) {
        distance += (point.position_W - last_point).norm();
      }
      last_point = point.position_W;
    }

    return distance;
}


void PathPlanner::loop()
{
    // TODO: Maybe re-plan while moving??
}

} // namespace voxblox
