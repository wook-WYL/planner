/**
 * @file sensor_coverage_planner_ground.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>
// ROS
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2/transform_datatypes.h>
// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
// Third parties
#include <utils/misc_utils.h>
#include <utils/pointcloud_utils.h>
// Components
#include "exploration_path/exploration_path.h"
#include "grid_world/grid_world.h"
#include "keypose_graph/keypose_graph.h"
#include "local_coverage_planner/local_coverage_planner.h"
#include "planning_env/planning_env.h"
#include "rolling_occupancy_grid/rolling_occupancy_grid.h"
#include "tare_visualizer/tare_visualizer.h"
#include "viewpoint_manager/viewpoint_manager.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

namespace sensor_coverage_planner_3d_ns {
const std::string kWorldFrameID = "map";
typedef pcl::PointXYZRGBNormal PlannerCloudPointType;
typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;
typedef misc_utils_ns::Timer Timer;

class SensorCoveragePlanner3D : public rclcpp::Node {
public:
  explicit SensorCoveragePlanner3D();
  bool initialize();
  void execute();
  ~SensorCoveragePlanner3D() = default;

private:
  // Parameters
  // String
  std::string sub_start_exploration_topic_;
  std::string sub_keypose_topic_;
  std::string sub_state_estimation_topic_;
  std::string sub_registered_scan_topic_;
  std::string sub_terrain_map_topic_;
  std::string sub_terrain_map_ext_topic_;
  std::string sub_coverage_boundary_topic_;
  std::string sub_viewpoint_boundary_topic_;
  std::string sub_nogo_boundary_topic_;
  std::string sub_joystick_topic_;
  std::string sub_reset_waypoint_topic_;

  std::string pub_exploration_finish_topic_;
  std::string pub_runtime_breakdown_topic_;
  std::string pub_runtime_topic_;
  std::string pub_waypoint_topic_;
  std::string pub_momentum_activation_count_topic_;

  // Bool
  bool kAutoStart;
  bool kRushHome;
  bool kUseTerrainHeight;
  bool kCheckTerrainCollision;
  bool kExtendWayPoint;
  bool kUseLineOfSightLookAheadPoint;
  bool kNoExplorationReturnHome;
  bool kUseMomentum;

  // Double
  double kKeyposeCloudDwzFilterLeafSize;
  double kRushHomeDist;
  double kAtHomeDistThreshold;
  double kTerrainCollisionThreshold;
  double kLookAheadDistance;
  double kExtendWayPointDistanceBig;
  double kExtendWayPointDistanceSmall;

  // Int
  int kDirectionChangeCounterThr;
  int kDirectionNoChangeCounterThr;
  int kResetWaypointJoystickAxesID;

  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>
      keypose_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>
      registered_scan_stack_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      registered_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      large_terrain_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      terrain_collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      terrain_ext_collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      viewpoint_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      grid_world_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      selected_viewpoint_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      exploring_cell_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      exploration_path_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      lookahead_point_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      keypose_graph_vis_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      viewpoint_in_collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      point_cloud_manager_neighbor_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
      reordered_global_subspace_cloud_;

  nav_msgs::msg::Odometry keypose_;
  geometry_msgs::msg::Point robot_position_;
  geometry_msgs::msg::Point last_robot_position_;
  lidar_model_ns::LiDARModel robot_viewpoint_;
  exploration_path_ns::ExplorationPath exploration_path_;
  Eigen::Vector3d lookahead_point_;
  Eigen::Vector3d lookahead_point_direction_;
  Eigen::Vector3d moving_direction_;
  double robot_yaw_;
  bool moving_forward_;
  std::vector<Eigen::Vector3d> visited_positions_;
  int cur_keypose_node_ind_;
  Eigen::Vector3d initial_position_;

  std::shared_ptr<keypose_graph_ns::KeyposeGraph> keypose_graph_;
  std::shared_ptr<planning_env_ns::PlanningEnv> planning_env_;
  std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager_;
  std::shared_ptr<local_coverage_planner_ns::LocalCoveragePlanner>
      local_coverage_planner_;
  std::shared_ptr<grid_world_ns::GridWorld> grid_world_;
  std::shared_ptr<tare_visualizer_ns::TAREVisualizer> visualizer_;

  std::shared_ptr<misc_utils_ns::Marker> keypose_graph_node_marker_;
  std::shared_ptr<misc_utils_ns::Marker> keypose_graph_edge_marker_;
  std::shared_ptr<misc_utils_ns::Marker> nogo_boundary_marker_;
  std::shared_ptr<misc_utils_ns::Marker> grid_world_marker_;

  bool keypose_cloud_update_;
  bool initialized_;
  bool lookahead_point_update_;
  bool relocation_;
  bool start_exploration_;
  bool exploration_finished_;
  bool near_home_;
  bool at_home_;
  bool stopped_;
  bool test_point_update_;
  bool viewpoint_ind_update_;
  bool step_;
  bool use_momentum_;
  bool lookahead_point_in_line_of_sight_;
  bool reset_waypoint_;
  pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZ> pointcloud_downsizer_;

  int update_representation_runtime_;
  int local_viewpoint_sampling_runtime_;
  int local_path_finding_runtime_;
  int global_planning_runtime_;
  int trajectory_optimization_runtime_;
  int overall_runtime_;
  int registered_cloud_count_;
  int keypose_count_;
  int direction_change_count_;
  int direction_no_change_count_;
  int momentum_activation_count_;

  double start_time_;
  double global_direction_switch_time_;
  double reset_waypoint_joystick_axis_value_;

  rclcpp::TimerBase::SharedPtr execution_timer_;

  // ROS subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      registered_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      terrain_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      terrain_map_ext_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      state_estimation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
      coverage_boundary_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
      viewpoint_boundary_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
      nogo_boundary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_waypoint_sub_;

  // ROS publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_full_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr old_global_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      to_nearest_global_subspace_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_tsp_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exploration_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_finish_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      runtime_breakdown_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr runtime_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
      momentum_activation_count_pub_;
  // Debug
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pointcloud_manager_neighbor_cells_origin_pub_;

  void ReadParameters();
  void InitializeData();

  // Callback functions
  void
  ExplorationStartCallback(const std_msgs::msg::Bool::ConstSharedPtr start_msg);
  void StateEstimationCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_msg);
  void RegisteredScanCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_cloud_msg);
  void TerrainMapCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg);
  void TerrainMapExtCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr
                                 terrain_cloud_large_msg);
  void CoverageBoundaryCallback(
      const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg);
  void ViewPointBoundaryCallback(
      const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg);
  void NogoBoundaryCallback(
      const geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_msg);
  void JoystickCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg);
  void
  ResetWaypointCallback(const std_msgs::msg::Empty::ConstSharedPtr empty_msg);

  void SendInitialWaypoint();
  void UpdateKeyposeGraph();
  int UpdateViewPoints();
  void UpdateViewPointCoverage();
  void UpdateRobotViewPointCoverage();
  void UpdateCoveredAreas(int &uncovered_point_num,
                          int &uncovered_frontier_point_num);
  void UpdateVisitedPositions();
  void UpdateGlobalRepresentation();
  void GlobalPlanning(std::vector<int> &global_cell_tsp_order,
                      exploration_path_ns::ExplorationPath &global_path);
  void PublishGlobalPlanningVisualization(
      const exploration_path_ns::ExplorationPath &global_path,
      const exploration_path_ns::ExplorationPath &local_path);
  void LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                     const exploration_path_ns::ExplorationPath &global_path,
                     exploration_path_ns::ExplorationPath &local_path);
  void PublishLocalPlanningVisualization(
      const exploration_path_ns::ExplorationPath &local_path);
  exploration_path_ns::ExplorationPath ConcatenateGlobalLocalPath(
      const exploration_path_ns::ExplorationPath &global_path,
      const exploration_path_ns::ExplorationPath &local_path);

  void PublishRuntime();
  double GetRobotToHomeDistance();
  void PublishExplorationState();
  void PublishWaypoint();
  bool
  GetLookAheadPoint(const exploration_path_ns::ExplorationPath &local_path,
                    const exploration_path_ns::ExplorationPath &global_path,
                    Eigen::Vector3d &lookahead_point);

  void PrintExplorationStatus(std::string status, bool clear_last_line = true);
  void CountDirectionChange();
};

} // namespace sensor_coverage_planner_3d_ns
