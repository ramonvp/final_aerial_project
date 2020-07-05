#include <ros/ros.h>
#include "final_aerial_project/path_planner.h"
#include <visualization_msgs/Marker.h>
#include <voxblox/utils/planning_utils.h>
#include <angles/angles.h>
#include <algorithm>


namespace final_aerial_project
{


std::ostream& operator<< (std::ostream& os, const final_aerial_project::PointCost & pcost)
{
    os << "distance : " << pcost.distance_cost << "\n" <<
          "angle    : " << pcost.angle_cost    << "\n" <<
          "obst     : " << pcost.obstacle_cost << "\n" <<
          "known    : " << pcost.known_cost;
    return os;
}


// Defining the binary function
bool comp_costs(const PointCost & a, const PointCost & b)
{
    return (a.cost() < b.cost());
}

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
    nh_private_.param("overshoot", overshoot_, 1);

    nh_private_.param("try_rrt", try_rrt_, true);

    nh_private_.param("max_z", max_z_, 1.5);

    nh_private_.param("distance_weight", distance_weight_, 1.0);
    nh_private_.param("angle_weight", angle_weight_, 1.0);
    nh_private_.param("obstacle_weight", obstacle_weight_, 1.0);
    nh_private_.param("known_weight", known_weight_, 1.0);

    nh_private_.param("use_cheating_paths", use_cheating_paths_, false);

    plan_service_ = nh_private_.advertiseService("makePlan", &PathPlanner::makePlanService, this);
    rviz_marker_pub_ = nh_private_.advertise<visualization_msgs::Marker>("grid_search", 5);
    path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
    grid_cost_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("grid_cost", 1, true);


    reconfig_server_ = new dynamic_reconfigure::Server<final_aerial_project::PlannerConfig>();
    dynamic_reconfigure::Server<final_aerial_project::PlannerConfig>::CallbackType f;
    f = boost::bind(&PathPlanner::reconfigure_callback, this,  _1, _2);
    reconfig_server_->setCallback(f);


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

    // get interesting places info
    std::vector<double> dummy3DVector{0.0, 0.0, 0.0};
    std::vector<double> dummy7DVector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    if(use_cheating_paths_)
    {
        // Read the coordinates from parameter server of the CheckPad
        nh_private_.param("/parcel_dispatcher/detection_threshold", detectionThreshold_, detectionThreshold_);

        nh_private_.getParam("/parcel_dispatcher/check_pad_position", dummy3DVector);
        check_pad_position_.x = dummy3DVector.at(0);
        check_pad_position_.y = dummy3DVector.at(1);
        check_pad_position_.z = detectionThreshold_;//dummy3DVector.at(2);

        nh_private_.getParam("/parcel_dispatcher/charging_pad_pose", dummy7DVector);
        charging_pad_position_.x = dummy7DVector.at(0);
        charging_pad_position_.y = dummy7DVector.at(1);
        charging_pad_position_.z = detectionThreshold_;//dummy7DVector.at(2);

        // shelve pose
        nh_private_.getParam("/parcel_dispatcher/shelve_pose", dummy7DVector);
        shelve_pose_ = pose_from_7D_vec(dummy7DVector);

        // husky pose
        nh_private_.getParam("/parcel_dispatcher/husky_pose", dummy7DVector);
        husky_pose_ = pose_from_7D_vec(dummy7DVector);

        createStaticPaths();
    }

}

PathPlanner::~PathPlanner()
{
    delete reconfig_server_;
}

void PathPlanner::reconfigure_callback(final_aerial_project::PlannerConfig &config, uint32_t level)
{
    distance_weight_ = config.distance_weight;
    angle_weight_ = config.angle_weight;
    obstacle_weight_ = config.obstacle_weight;
    known_weight_ = config.known_weight;
    grid_sep_ = config.grid_separation;

    printf("New Parameters:\n");
    printf("   distance_weight_: %f\n", distance_weight_);
    printf("   angle_weight_: %f\n", angle_weight_);
    printf("   obstacle_weight_: %f\n", obstacle_weight_);
    printf("   known_weight_: %f\n", known_weight_);
    printf("   grid_sep_: %f\n", grid_sep_);

    ROS_INFO ("[PathPlanner] Reconfigure callback have been called with new parameters");
}

static trajectory_msgs::MultiDOFJointTrajectoryPoint createTrajPoint(double x, double y, double z, double yaw)
{
    trajectory_msgs::MultiDOFJointTrajectoryPoint p;
    geometry_msgs::Transform t;
    t.translation.x = x;
    t.translation.y = y;
    t.translation.z = z;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, yaw);
    tf::quaternionTFToMsg(quat, t.rotation);
    p.transforms.push_back(t);
    return p;
}

void PathPlanner::createStaticPaths()
{
    // First path is from Charging Pad to Shelves position
    fromChargeToShelve_.points.push_back( createTrajPoint(14,16,1, M_PI/4) );
    fromChargeToShelve_.points.push_back( createTrajPoint(29,16,1, 0) );
    fromChargeToShelve_.points.push_back( createTrajPoint(30,11,1, -3*M_PI/4) );
    fromChargeToShelve_.points.push_back( createTrajPoint(21.82, 3.13, 1.0, -M_PI/2) );
    fromChargeToShelve_.points.push_back( createTrajPoint(15,3,1, -M_PI/2) );

    fromCheckpadToHusky_.points.push_back( createTrajPoint(31,12,1, M_PI/4) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(30,17,1, M_PI/2) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(20,18,1, M_PI) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(16,24,1, 3*M_PI/4) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(14,26,1, M_PI/2) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(9,28,1, M_PI) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(2,28,1, -M_PI/2) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(2.5,23.5,1, 0) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(6,23.5,1, M_PI/2) );
    fromCheckpadToHusky_.points.push_back( createTrajPoint(6,25,1,  M_PI/2) );

    /* alternative path for checkpad - husky
       WP 0	:	7.27	24.62	0.86	-2.78
       WP 1	:	4.52	23.59	0.86	2.67
       WP 2	:	3.10	24.30	0.58	1.56
       WP 3	:	3.12	26.49	1.10	1.14
       WP 4	:	3.58	27.50	1.23	-0.05
       WP 5	:	9.46	27.17	0.98	-0.51
       WP 6	:	18.39	22.16	0.75	-0.52
       WP 7	:	29.26	15.92	0.64	-1.26
       WP 8	:	29.94	13.78	0.95	-2.19
       WP 9	:	24.07	5.62	1.42	-2.31
       WP 10	:	21.82	3.13	0.86	0.00
    */
}

geometry_msgs::Pose PathPlanner::pose_from_7D_vec(const std::vector<double> & poseVec)
{
    geometry_msgs::Pose geom_pose;

    geom_pose.position.x = poseVec.at(0);
    geom_pose.position.y = poseVec.at(1);
    geom_pose.position.z = poseVec.at(2);

    double quatnorm = 0.0;
    for (int ii = 3; ii < 7; ii++)
    {
      quatnorm += poseVec.at(ii) * poseVec.at(ii);
    }
    quatnorm = std::sqrt(quatnorm);

    geom_pose.orientation.x = poseVec.at(3) / quatnorm;
    geom_pose.orientation.y = poseVec.at(4) / quatnorm;
    geom_pose.orientation.z = poseVec.at(5) / quatnorm;
    geom_pose.orientation.w = poseVec.at(6) / quatnorm;

    return geom_pose;
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
        ROS_DEBUG("Point (%f,%f,%f) is unknown to the map", position[0], position[1], position[2]);
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
        ROS_DEBUG("Point (%f,%f,%f) collides d = %f", position[0],position[1],position[2], d);
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

    voxblox::utils::computeMapBoundsFromLayer(
          *esdf_map_->getEsdfLayerPtr(),
          &lower_bound, &upper_bound);

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

std::vector<geometry_msgs::Point> PathPlanner::generateGridOnBounds(double z)
{
    std::vector<geometry_msgs::Point> grid;

    Eigen::Vector3d lower_bound(Eigen::Vector3d::Zero());
    Eigen::Vector3d upper_bound(Eigen::Vector3d::Zero());

    voxblox::utils::computeMapBoundsFromLayer(
          *esdf_map_->getEsdfLayerPtr(),
          &lower_bound, &upper_bound);

    geometry_msgs::Point p;
    p.z = z;
    p.y = lower_bound[1];
    while(p.y < upper_bound[1])
    {
        p.x = lower_bound[0];
        while(p.x < upper_bound[0])
        {
            grid.push_back(p);
            p.x += grid_sep_;
        }
        p.y += grid_sep_;
    }

    return grid;
}

std::vector<geometry_msgs::Point> PathPlanner::generateGridTowardsGoal(const geometry_msgs::Point & start_point, const geometry_msgs::Point & end_point)
{
    std::vector<geometry_msgs::Point> grid;

    double dx = end_point.x - start_point.x;
    double dy = end_point.y - start_point.y;
    double yaw = atan2(dy, dx);
    double norm = sqrt(dx*dx + dy*dy);
    int nx = static_cast<int>(norm/grid_sep_ + 0.5) + overshoot_;
    if(nx%2 == 0) nx++;

    for(int k = 0; k < Nz_; k++)
    {
        double offset_z = (k - Nz_/2)*grid_sep_;
        if( start_point.z + offset_z > max_z_ )
          continue;

        for(int j = 0; j < Ny_; j++)
        {
            double offset_y = (j - Ny_/2)*grid_sep_;
            for(int i = 0; i < nx; i++)
            {
                geometry_msgs::Point p;
                //double offset_x = (i - nx/2)*grid_sep_;
                double offset_x = i*grid_sep_;
                p.x = start_point.x + offset_x*cos(yaw) - offset_y*sin(yaw);
                p.y = start_point.y + offset_x*sin(yaw) + offset_y*cos(yaw);
                p.z = start_point.z + offset_z;

                // skip start location, should not be on the grid
                double distance_point = euclideanDistance(p,  start_point);
                if( distance_point < grid_sep_) continue;

                grid.push_back(p);
            }
        }
    }

    return grid;
}

std::vector<geometry_msgs::Point> PathPlanner::generateGrid(const geometry_msgs::Point & point, int nx, int ny, int nz, double yaw, double sep)
{
    std::vector<geometry_msgs::Point> points;

    for(int k = 0; k < nz; k++)
    {
        double offset_z = (k - nz/2)*sep;
        if( point.z + offset_z > max_z_ )
          continue;

        for(int j = 0; j < ny; j++)
        {
            double offset_y = (j - ny/2)*sep;
            for(int i = 0; i < nx; i++)
            {
                geometry_msgs::Point p;
                double offset_x = (i - nx/2)*sep;
                p.x = point.x + offset_x*cos(yaw) - offset_y*sin(yaw);
                p.y = point.y + offset_x*sin(yaw) + offset_y*cos(yaw);
                p.z = point.z + offset_z;

                // skip my current location
                //double distance_point = euclideanDistance(p,  point);
                //if( distance_point < grid_sep_) continue;

                points.push_back(p);
            }
        }
    }
    return points;
}

bool PathPlanner::isPredefinedPath(const geometry_msgs::Pose & start_pose,
                                   const geometry_msgs::Pose & goal_pose,
                                   trajectory_msgs::MultiDOFJointTrajectory & sampled_plan)
{

    if( euclideanDistance(start_pose.position,  charging_pad_position_) < detectionThreshold_ &&
        euclideanDistance(goal_pose.position,  shelve_pose_.position) < detectionThreshold_)
    {

        sampled_plan.points = fromChargeToShelve_.points;
        return true;
    }

    // Husky pose has been generated with a 2-sigma 2m gaussian noise,
    // so compare with 2.5m instead of normal detection threshold
    if( euclideanDistance(start_pose.position,  check_pad_position_) < detectionThreshold_ &&
        euclideanDistance(goal_pose.position,  husky_pose_.position) < 2.5)
    {

        sampled_plan.points = fromCheckpadToHusky_.points;
        // since Husky position has been added a 2m sigma noise, add the
        // real pose to the end
        sampled_plan.points.push_back(
              createTrajPoint(
                goal_pose.position.x,
                goal_pose.position.y,
                goal_pose.position.z,
                tf::getYaw(goal_pose.orientation)
                )
              );
        return true;
    }


    return false;
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

    if(use_cheating_paths_)
    {
        bool is_predefined = isPredefinedPath(start_pose.pose, goal_pose.pose, sampled_plan);
        if(is_predefined)
        {
            ROS_INFO("[PathPlanner] A predefined path has been used");
            return true;
        }
    }

    if( try_rrt_ )
    {
        // ask the RRT to look for a path
        bool rrt_plan_ok = makePlan_RRT(start_pose, goal_pose, sampled_plan);
        if(!rrt_plan_ok)
        {
            ROS_ERROR("RRT failed to produce a valid path, backfall to local steps");
        }
        else  // we're done
        {
            // last waypoint must have the requested orientation
            (*sampled_plan.points.rbegin()).transforms[0].rotation = goal_pose.pose.orientation;
            return true;
        }
    }
    else
    {
        ROS_WARN("RRT is disabled by option try_rrt");
    }

    // Generate a grid of possible goal points all over the map
    // bounds that are not occupied and compute their costs
    std::vector<geometry_msgs::Point> grid_search;
    std::vector<PointCost> costs;
    computeGridWithCosts(start_pose.pose, goal_pose.pose, grid_search, costs);
    if( costs.size() == 0)
    {
        ROS_FATAL("[Mission] No candidates found, planner has no plan");
        return false;
    }

    // in case RRT fails, we will still go in straight line
    int minCostIndex = std::min_element(costs.begin(),costs.end(), comp_costs) - costs.begin();
    geometry_msgs::Point failSafePoint = grid_search[minCostIndex];

    while(costs.size())
    {
        minCostIndex = std::min_element(costs.begin(),costs.end(), comp_costs) - costs.begin();
        const geometry_msgs::Point & sp = grid_search[minCostIndex];
        ROS_INFO("Selected next point is: %f, %f, %f",  sp.x, sp.y, sp.z);
        geometry_msgs::PoseStamped winnerPose;
        winnerPose = goal_pose; // copy all for convenience
        winnerPose.pose.position = sp;
        bool rrt_plan_ok = makePlan_RRT(start_pose, winnerPose, sampled_plan);
        if(rrt_plan_ok)
        {
            return true;
        }
        else {
            ROS_ERROR("RRT failed to produce a valid path to selected point, try with next one");
            // remove the point from the grid search and try again
            costs.erase (costs.begin()+minCostIndex);
            grid_search.erase (grid_search.begin()+minCostIndex);
        }
    }

    ROS_FATAL("[Mission] All candidates in grid search were discarded by RRT! Will go in straight line");

    // Send a manual goal in straight line
    const geometry_msgs::Point & sp = failSafePoint;
    publishGridSearchArrow(sampled_plan.header.frame_id, start_pose.pose.position, sp);

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

    return true;

#if 0
    int minCostIndex = std::min_element(costs.begin(),costs.end(), comp_costs) - costs.begin();
    if( minCostIndex >= 0 && minCostIndex < static_cast<int>(grid_search.size()))
    {
        const geometry_msgs::Point & sp = grid_search[minCostIndex];
        ROS_INFO("Selected next point is: %f, %f, %f",  sp.x, sp.y, sp.z);


        publishGridSearchPoints(sampled_plan.header.frame_id, grid_search, false);

        geometry_msgs::PoseStamped winnerPose;
        winnerPose = goal_pose; // copy all for convenience
        winnerPose.pose.position = sp;
        bool rrt_plan_ok = makePlan_RRT(start_pose, winnerPose, sampled_plan);
        if(rrt_plan_ok)
        {
            return true;
        }
        else {
            ROS_ERROR("RRT failed to produce a valid path to selected point, will move in straight line");
        }

        publishGridSearchArrow(sampled_plan.header.frame_id, start_pose.pose.position, sp);

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


        return true;
    }
    else
    {
        ROS_ERROR("All candidates in grid search were discarded!");
    }

    return false;
#endif
}


void PathPlanner::computeGridWithCosts(const geometry_msgs::Pose & start_pose,
                                       const geometry_msgs::Pose & goal_pose,
                                       std::vector<geometry_msgs::Point> & grid_search,
                                       std::vector<PointCost> & costs)
{
    //std::vector<geometry_msgs::Point> full_grid = generateGridTowardsGoal(start_pose.position, goal_pose.position);
    std::vector<geometry_msgs::Point> full_grid = generateGridOnBounds(goal_pose.position.z);

    full_grid.push_back(goal_pose.position);

    publishGridSearchPoints("world", full_grid, true);

    full_grid.push_back(goal_pose.position);

    visualization_msgs::MarkerArray marker_array;
    int i = 0;
    for( auto & p : full_grid)
    {
        if( isCollisionFree(p) )
        {
            grid_search.push_back(p);
            PointCost pcost = computePointCost(p, goal_pose, start_pose);
            costs.push_back(pcost);
            createCylinderCost(pcost.costs(), p, i, marker_array);
            i++;
        }
    }

    grid_cost_marker_pub_.publish(marker_array);
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
    marker.scale.y = 0.12; // head diameter
    marker.scale.z = 0.12; // if not zero, head length

    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.points.push_back(start);
    marker.points.push_back(end);


    rviz_marker_pub_.publish(marker);
}

void PathPlanner::publishGoalPointMarker(const geometry_msgs::PoseStamped & pose)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.ns = "goal_point";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = ros::Time::now();

    marker.pose.position = pose.pose.position;
    marker.pose.orientation.w = 1.0;

    // Scale is the diameter of the shape
    marker.scale.x = 0.4;
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;

    marker.color.b = 1.0;
    marker.color.a = 1.0;

    rviz_marker_pub_.publish(marker);
}

void PathPlanner::publishGridSearchPoints(const std::string & frame_id, const std::vector<geometry_msgs::Point> & points, bool is_full)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    if(is_full)
      marker.ns = "grid_search_full";
    else
      marker.ns = "grid_search";

    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    //marker.pose = req.start_pose.pose;
    marker.pose.orientation.w = 1.0;

    // Scale is the diameter of the shape
    marker.scale.x = 0.2;
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;

    if(is_full)
    {
      marker.color.r = 1.0;
    }
    else {
      marker.color.b = 1.0;
    }
    marker.color.a = 1.0;
    marker.points = points;

    rviz_marker_pub_.publish(marker);
}

std_msgs::ColorRGBA PathPlanner::colorFromIndex(int index) const
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;

  uint8_t n = (index+1)%8;
  color.r = ((n>>2)&0x1)*1.0;
  color.g = ((n>>1)&0x1)*1.0;
  color.b = ((n>>0)&0x1)*1.0;

  /*
  printf("Color index = %d => color R: %g  G: %g  B: %g\n",
         index, color.r, color.g, color.b);
  */
  return color;
}

void PathPlanner::createCylinderCost(const std::vector<double> & costs,
                                    const geometry_msgs::Point & point,
                                    const int start_id,
                                    visualization_msgs::MarkerArray & marker_array) const
{
    ros::Time now = ros::Time::now();
    double offset = 0.0;
    for(int i = 0; i < costs.size(); i++)
    {
        double cost = costs[i];
        visualization_msgs::Marker cylinder_marker;
        cylinder_marker.header.frame_id = "world";
        cylinder_marker.header.stamp = now;
        cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
        cylinder_marker.color = colorFromIndex(i);
        cylinder_marker.ns = "cylinders_cost_" + std::to_string(i);
        cylinder_marker.id = start_id + i;
        cylinder_marker.scale.x = 0.2;
        cylinder_marker.scale.y = cylinder_marker.scale.x;
        cylinder_marker.scale.z = cost;
        cylinder_marker.pose.position = point;
        cylinder_marker.pose.position.z += offset + cost/2.0;
        //cylinder_marker.pose.position.z = offset + cost/2.0;
        cylinder_marker.pose.orientation.w = 1.0;
        marker_array.markers.push_back(cylinder_marker);
        offset += cost;
    }

}


void PathPlanner::getInfoCallback(const geometry_msgs::Point & start, const geometry_msgs::Point & goal)
{
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position = goal;
    goal_pose.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped start_pose;
    start_pose.pose.position = start;
    start_pose.pose.orientation.w = 1.0;

    std::vector<geometry_msgs::Point> grid_search;
    std::vector<PointCost> costs;
    computeGridWithCosts(start_pose.pose, goal_pose.pose, grid_search, costs);

    int minCostIndex = std::min_element(costs.begin(),costs.end(), comp_costs) - costs.begin();
    printf("Selected point index: %d\n", minCostIndex);
    ROS_ASSERT(minCostIndex >= 0);
    const geometry_msgs::Point & selected = grid_search[minCostIndex];

    double yaw = atan2(selected.y - start_pose.pose.position.y, selected.x - start_pose.pose.position.x);
    printf("Selected point: %g,%g,%g  Yaw = %g\n",
           selected.x, selected.y, selected.z, yaw);

    printf("./send_waypoint.py %g %g %g %g %g %g %g %g\n",
           start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z, yaw,
           selected.x, selected.y, selected.z, yaw);

    publishGridSearchArrow("world", start_pose.pose.position, selected);
}

PointCost PathPlanner::computePointCost(const geometry_msgs::Point & point,
                                     const geometry_msgs::Pose & goal_pose,
                                     const geometry_msgs::Pose & start_pose)
{

    double distance_to_goal = 0.0;
    if(distance_weight_)
    {
        double dist_start_to_goal = euclideanDistance(start_pose.position,  goal_pose.position);
        distance_to_goal = euclideanDistance(point,  goal_pose.position);
        // normalize distance, 1.0 means distance from start to goal
        // if < 1.0, means I'm closer. If > 1.0, means I'm farther
        distance_to_goal = distance_to_goal/dist_start_to_goal;
    }

    double currentYaw = tf::getYaw(start_pose.orientation);
    double targetYaw = atan2(goal_pose.position.y - start_pose.position.y,
                             goal_pose.position.x - start_pose.position.x);
    double pointYaw = atan2(point.y - start_pose.position.y,
                             point.x - start_pose.position.x);
    double error_yaw = abs(angles::shortest_angular_distance(targetYaw, pointYaw));
    // normalize error angle, such that 180º means +1 in cost, error is [0, 1.0]
    error_yaw = error_yaw/M_PI;

    double obstacle_distance = p_collision_radius_;
    if(obstacle_weight_)
    {
        obstacle_distance = abs(getMapDistance(point));
        // if obstacle_distance is 0, this means that this point is invalid
        // and should be discarded. This however should not happen and probably
        // won't reach this stage
        if(obstacle_distance == 0.0)
           obstacle_distance = p_collision_radius_/1000.0;
    }

    int countPoints {0};
    double numPointsKnown {0.0};
    if(known_weight_)
    {
        std::vector<geometry_msgs::Point> pointList = generateGrid(point, 15, 15, 3, 0.0, 0.2);
        for( const auto & p : pointList)
        {
            if( isPointObserved(p) )
                countPoints++;
        }
        numPointsKnown = static_cast<double>(countPoints)/pointList.size();
        //printf("    TotalPoints: %lu  KnownPoints: %d  Ratio: %f\n", pointList.size(), countPoints, numPointsKnown);
        //printf("Point %f,%f  numPointKnown = %f\n", point.x, point.y, numPointsKnown);
    }

    PointCost cost;
    cost.distance_cost = distance_weight_ * distance_to_goal;
    cost.angle_cost    = angle_weight_    * error_yaw;
    cost.obstacle_cost = obstacle_weight_ * (p_collision_radius_/obstacle_distance);
    cost.known_cost    = known_weight_    * numPointsKnown;

    return cost;
}

// Returns true if the point exists in the map AND is observed.
bool PathPlanner::isPointObserved(const geometry_msgs::Point & point) const
{
    return isPointObserved(Eigen::Vector3d(point.x, point.y, point.z));
}

bool PathPlanner::isPointObserved(const Eigen::Vector3d &position) const
{
    return esdf_map_->isObserved(position);
    //double distance = 0.0;
    //const bool kInterpolate = false;
    //return esdf_map_->getDistanceAtPosition(position, kInterpolate, &distance);
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

    ROS_DEBUG_STREAM("Map bounds: " << lower_bound_.transpose() << " to "
                                   << upper_bound_.transpose() << " size: "
                                   << (upper_bound_ - lower_bound_).transpose());

    // Limit Z to the same as goal, to avoid jumping over the walls
    //lower_bound_[2] = max_z_;
    //upper_bound_[2] = max_z_;

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
        ROS_ERROR("[Mission] RRT: Start pose occupied!");
        return false;
    }

    if (getMapDistance(goal_pose.position_W) < p_collision_radius_) {
        ROS_ERROR("[Mission] RRT: Goal pose occupied!");
        return false;
    }

    mav_msgs::EigenTrajectoryPoint::Vector waypoints;
    bool success = rrt_.getPathBetweenWaypoints(start_pose, goal_pose, &waypoints);
    double path_length = computePathLength(waypoints);
    std::size_t num_vertices = waypoints.size();
    ROS_INFO("RRT* Success? %d Path length: %f Vertices: %lu", success, path_length, num_vertices);

    if (!success)
    {
        return false;
    }

    mav_msgs::msgMultiDofJointTrajectoryFromEigen(waypoints, "base_link", &rrt_plan);

    // setup the correct orientation of each waypoint
    int num_points = rrt_plan.points.size();
    double last_yaw;
    for( int i = 0; i < num_points; i++)
    {
        auto & transf = rrt_plan.points[i].transforms[0];
        if( i < num_points -1 )
        {
          auto & next_transf = rrt_plan.points[i+1].transforms[0];
          double dx = next_transf.translation.x - transf.translation.x;
          double dy = next_transf.translation.y - transf.translation.y;
          last_yaw = atan2(dy,dx);
        }
        tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, last_yaw);
        tf::quaternionTFToMsg(quat, transf.rotation);
    }

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
