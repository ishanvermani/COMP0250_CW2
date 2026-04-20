/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire
solution is contained within the cw2_team_<your_team_number> package */

#ifndef CW2_CLASS_H_
#define CW2_CLASS_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "cw2_world_spawner/srv/task1_service.hpp"
#include "cw2_world_spawner/srv/task2_service.hpp"
#include "cw2_world_spawner/srv/task3_service.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <Eigen/Core>
#include <cmath>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

class cw2
{
public:
  explicit cw2(const rclcpp::Node::SharedPtr &node);

  void t1_callback(
    const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
    std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> response);
  void t2_callback(
    const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
    std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response);
  void t3_callback(
    const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> request,
    std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response);

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  
  void rosTopicToCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_input_msg);
  void applyVoxelGrid(double leaf_size);
  void applyPassthrough(double pass_min, double pass_max, std::string pass_axis);
  void applyOutlierRemoval(int mean_k, double stddev);
  void findNormals(int normal_k);
  void segmentationPipeline(double normal_dist_weight, int max_iterations, double distance);
  std::vector<PointCPtr> extractEuclideanClusters(double cluster_tolerance, int min_size, int max_size);
  void pubFilteredPCMsg(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pc_pub, PointC &pc, const std_msgs::msg::Header &header);
  void processCloud();
  Eigen::Vector3f getCentroid(PointC &in_cloud_ptr);
  std::string colorOfPointCloud(PointC &in_cloud_ptr, float threshold);
  void filteringPipeline();
  Eigen::Vector3f toWorldFrame(Eigen::Vector3f local_point);
  bool moveToBirdeye(moveit::planning_interface::MoveGroupInterface &move_group, float theta);



  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<cw2_world_spawner::srv::Task1Service>::SharedPtr t1_service_;
  rclcpp::Service<cw2_world_spawner::srv::Task2Service>::SharedPtr t2_service_;
  rclcpp::Service<cw2_world_spawner::srv::Task3Service>::SharedPtr t3_service_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr color_cloud_sub_;
  rclcpp::CallbackGroup::SharedPtr pointcloud_callback_group_;

    //debug publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_passthrough;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_outlier;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_plane;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_cluster1;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_cluster2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_cluster3;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_cluster4;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_cluster5;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_cluster6;
  std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, 6> g_pub_clusters;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_msg_;

  //pcl variables
  PointCPtr g_cloud_ptr;
  PointCPtr g_cloud_filtered;
  PointCPtr g_cloud_plane;
  PointCPtr g_cloud_segmented_plane;
  PointCPtr g_cloud_cluster;

  std::string g_input_pc_frame_id;

  pcl::VoxelGrid<PointT> g_vx;
  pcl::PassThrough<PointT> g_pt;
  pcl::StatisticalOutlierRemoval<PointT> g_sor;

  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
  pcl::search::KdTree<PointT>::Ptr g_tree_ptr_euclidean;
  pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals;
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_segmented_normals;

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg;
  pcl::ExtractIndices<PointT> g_extract_pc;
  pcl::ExtractIndices<pcl::Normal> g_extract_normals;
  pcl::EuclideanClusterExtraction<PointT> g_extract_euclidean;
  pcl::MomentOfInertiaEstimation<PointT> g_inertia_estimator;

  pcl::PointIndices::Ptr g_inliers_plane;
  pcl::ModelCoefficients::Ptr g_coeff_plane;

  std::atomic<int64_t> latest_joint_state_stamp_ns_{0};
  std::atomic<uint64_t> joint_state_msg_count_{0};
  std::atomic<int64_t> latest_cloud_stamp_ns_{0};
  std::atomic<uint64_t> cloud_msg_count_{0};

  bool enable_cloud_viewer_ = false;
  bool move_home_on_start_ = false;
  bool use_path_constraints_ = false;
  bool use_cartesian_reach_ = false;
  bool allow_position_only_fallback_ = false;
  bool publish_programmatic_debug_ = false;
  bool enable_task1_snap_ = false;
  bool return_home_between_pick_place_ = false;
  bool return_home_after_pick_place_ = false;
  bool task2_capture_enabled_ = false;

  double cartesian_eef_step_ = 0.005;
  double cartesian_jump_threshold_ = 0.0;
  double cartesian_min_fraction_ = 0.98;
  double pick_offset_z_ = 0.12;
  double task3_pick_offset_z_ = 0.13;
  double place_offset_z_ = 0.35;
  double grasp_approach_offset_z_ = 0.015;
  double post_grasp_lift_z_ = 0.05;
  double gripper_grasp_width_ = 0.02;
  double joint_state_wait_timeout_sec_ = 2.0;

  //hardcode debug values
  double pcl_voxel_leaf_size_      = 0.01;
  double pcl_pass_min_             = 0.18;
  double pcl_pass_max_             = -0.30;
  std::string pcl_pass_axis_       = "y";
  int    pcl_outlier_mean_k_       = 20;
  double pcl_outlier_stddev_       = 1.0;
  int    pcl_normal_k_             = 50;
  double pcl_plane_normal_weight_  = 0.1;
  int    pcl_plane_max_iterations_ = 100;
  double pcl_plane_distance_       = 0.03;
  double pcl_cluster_tolerance_    = 0.02;
  int    pcl_cluster_min_size_     = 5000;
  int    pcl_cluster_max_size_     = 60000;


  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex cloud_mutex_;
  std::uint64_t g_cloud_sequence_ = 0;
  std::string g_input_pc_frame_id_;

  std::string pointcloud_topic_;
  bool pointcloud_qos_reliable_ = false;

  // Create an array of all colors

  static constexpr size_t num_colors = 5;
  const std::array<std::array<float, 3>, num_colors> colors = {{
      {0.1f, 0.1f, 0.8f},   // blue
      {0.8f, 0.1f, 0.8f},   // purple
      {0.8f, 0.1f, 0.1f},   // red
      {0.1f, 0.1f, 0.1f},   //black
      {0.5f, 0.2f, 0.2f}    //brown

  }};  
  
  const std::array<std::string, num_colors> color_names = {"blue", "purple", "red", "black", "brown"};

  const std::string no_color = "none";

  // struct and enum to define shapes

  enum class SHAPE_TYPE
  {
    NOUGHT,
    CROSS
  };

  enum class SHAPE_SIZE
  {
    MM_20,
    MM_30,
    MM_40
  };

  struct SHAPE
  {
    SHAPE_TYPE type;
    SHAPE_SIZE size;
  };

  SHAPE classifyShape(PointC &in_cloud_ptr);


};

#endif  // CW2_CLASS_H_
