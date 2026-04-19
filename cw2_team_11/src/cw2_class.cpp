/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire
solution is contained within the cw2_team_<your_team_number> package */


/*

COMP0250 Coursework 2 - Team 11





*/

/**
 * KEY ASSUMPTIONS
 * * 
 * * */


#include <cw2_class.h>
#include <cmath>




#include <utility>

cw2::cw2(const rclcpp::Node::SharedPtr &node)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_)
{


//init values
  g_cloud_ptr = std::make_shared<PointC>();
  g_cloud_filtered = std::make_shared<PointC>();
  g_cloud_plane = std::make_shared<PointC>();
  g_cloud_segmented_plane = std::make_shared<PointC>();
  g_cloud_cluster = std::make_shared<PointC>();
  g_tree_ptr = std::make_shared<pcl::search::KdTree<PointT>>();
  g_tree_ptr_euclidean = std::make_shared<pcl::search::KdTree<PointT>>();
  g_cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  g_cloud_segmented_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  g_inliers_plane = std::make_shared<pcl::PointIndices>();
  g_coeff_plane = std::make_shared<pcl::ModelCoefficients>();

  // PCL DEBUG VALUES
  pcl_voxel_leaf_size_      = node_->declare_parameter("pcl.voxel_leaf_size",      pcl_voxel_leaf_size_);
  pcl_pass_min_             = node_->declare_parameter("pcl.pass_min",             pcl_pass_min_);
  pcl_pass_max_             = node_->declare_parameter("pcl.pass_max",             pcl_pass_max_);
  pcl_pass_axis_            = node_->declare_parameter("pcl.pass_axis",            pcl_pass_axis_);
  pcl_outlier_mean_k_       = node_->declare_parameter("pcl.outlier_mean_k",       pcl_outlier_mean_k_);
  pcl_outlier_stddev_       = node_->declare_parameter("pcl.outlier_stddev",       pcl_outlier_stddev_);
  pcl_normal_k_             = node_->declare_parameter("pcl.normal_k",             pcl_normal_k_);
  pcl_plane_normal_weight_  = node_->declare_parameter("pcl.plane_normal_weight",  pcl_plane_normal_weight_);
  pcl_plane_max_iterations_ = node_->declare_parameter("pcl.plane_max_iterations", pcl_plane_max_iterations_);
  pcl_plane_distance_       = node_->declare_parameter("pcl.plane_distance",       pcl_plane_distance_);
  pcl_cluster_tolerance_    = node_->declare_parameter("pcl.cluster_tolerance",    pcl_cluster_tolerance_);
  pcl_cluster_min_size_     = node_->declare_parameter("pcl.cluster_min_size",     pcl_cluster_min_size_);
  pcl_cluster_max_size_     = node_->declare_parameter("pcl.cluster_max_size",     pcl_cluster_max_size_);

  // UPDATE CALLBACK FOR PCL DEBUGGING
  param_cb_handle_= node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params)
    {

      RCLCPP_INFO_STREAM(node_->get_logger(), "Callback param reset triggered");

      for (const auto &p : params) {
        const auto &n = p.get_name();
        if      (n == "pcl.voxel_leaf_size")      pcl_voxel_leaf_size_      = p.as_double();
        else if (n == "pcl.pass_min")             pcl_pass_min_             = p.as_double();
        else if (n == "pcl.pass_max")             pcl_pass_max_             = p.as_double();
        else if (n == "pcl.pass_axis")            pcl_pass_axis_            = p.as_string();
        else if (n == "pcl.outlier_mean_k")       pcl_outlier_mean_k_       = static_cast<int>(p.as_int());
        else if (n == "pcl.outlier_stddev")       pcl_outlier_stddev_       = p.as_double();
        else if (n == "pcl.normal_k")             pcl_normal_k_             = static_cast<int>(p.as_int());
        else if (n == "pcl.plane_normal_weight")  pcl_plane_normal_weight_  = p.as_double();
        else if (n == "pcl.plane_max_iterations") pcl_plane_max_iterations_ = static_cast<int>(p.as_int());
        else if (n == "pcl.plane_distance")       pcl_plane_distance_       = p.as_double();
        else if (n == "pcl.cluster_tolerance")    pcl_cluster_tolerance_    = p.as_double();
        else if (n == "pcl.cluster_min_size")     pcl_cluster_min_size_     = static_cast<int>(p.as_int());
        else if (n == "pcl.cluster_max_size")     pcl_cluster_max_size_     = static_cast<int>(p.as_int());
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      processCloud();
      return result;
    });

  t1_service_ = node_->create_service<cw2_world_spawner::srv::Task1Service>(
    "/task1_start",
    std::bind(&cw2::t1_callback, this, std::placeholders::_1, std::placeholders::_2));
  t2_service_ = node_->create_service<cw2_world_spawner::srv::Task2Service>(
    "/task2_start",
    std::bind(&cw2::t2_callback, this, std::placeholders::_1, std::placeholders::_2));
  t3_service_ = node_->create_service<cw2_world_spawner::srv::Task3Service>(
    "/task3_start",
    std::bind(&cw2::t3_callback, this, std::placeholders::_1, std::placeholders::_2));

  //debug publishers
  g_pub_cloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud", 1);
  g_pub_passthrough = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_passthrough", 1);
  g_pub_outlier = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_outlier", 1);
  g_pub_plane = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_plane", 1);
  g_pub_cluster1 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_cluster1", 1);
  g_pub_cluster2 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_cluster2", 1);
  g_pub_cluster3 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_cluster3", 1);
  g_pub_cluster4 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_cluster4", 1);
  g_pub_cluster5 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_cluster5", 1);
  g_pub_cluster6 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw2_cloud/cloud_cluster6", 1);

  g_pub_clusters = {g_pub_cluster1, g_pub_cluster2, g_pub_cluster3, g_pub_cluster4, g_pub_cluster5, g_pub_cluster6};

  
  pointcloud_topic_ = node_->declare_parameter<std::string>(
    "pointcloud_topic", "/r200/camera/depth_registered/points");
  pointcloud_qos_reliable_ =
    node_->declare_parameter<bool>("pointcloud_qos_reliable", true);

  pointcloud_callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions pointcloud_sub_options;
  pointcloud_sub_options.callback_group = pointcloud_callback_group_;

  rclcpp::QoS pointcloud_qos = rclcpp::SensorDataQoS();
  if (pointcloud_qos_reliable_) {
    pointcloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  }

  color_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_,
    pointcloud_qos,
    std::bind(&cw2::cloud_callback, this, std::placeholders::_1),
    pointcloud_sub_options);

  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "panda_arm");
  hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand");

  RCLCPP_INFO(
    node_->get_logger(),
    "cw2_team_11 template initialised with pointcloud topic '%s' (%s QoS)",
    pointcloud_topic_.c_str(),
    pointcloud_qos_reliable_ ? "reliable" : "sensor-data");
}

void cw2::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);

  latest_cloud_msg_ = msg;

  PointCPtr latest_cloud(new PointC);
  pcl::fromPCLPointCloud2(pcl_cloud, *latest_cloud);

  *g_cloud_filtered = *latest_cloud;

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  g_input_pc_frame_id_ = msg->header.frame_id;
  g_cloud_ptr = std::move(latest_cloud);
  ++g_cloud_sequence_;
}


void cw2::t1_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> response)
{

  (void)response;
  auto L = node_->get_logger();

    RCLCPP_INFO(L, " Moving camera right above shape at (%.3f, %.3f, %.3f)",
              request->object_point.point.x,
              request->object_point.point.y,
              request->object_point.point.z);

  
  static const std::string planning_group = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group2(node_, planning_group);

  move_group2.setPlanningTime(5.0);
  move_group2.setMaxVelocityScalingFactor(0.2);
  move_group2.setMaxAccelerationScalingFactor(0.2);
  
  
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = request->object_point.point.x;
  target_pose.position.y = request->object_point.point.y;
  target_pose.position.z = request->object_point.point.z + 0.5;

  double roll = M_PI; // 180 degrees
  double pitch = 0;
  double yaw = - M_PI / 4; // -45 degrees

  //for the quaternions
  double half_roll = roll / 2.0;
  double half_pitch = pitch / 2.0;
  double half_yaw = yaw / 2.0;

  double w = std::cos(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) + std::sin(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
  double x = std::sin(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) - std::cos(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
  double y = std::cos(half_roll) * std::sin(half_pitch) * std::cos(half_yaw) + std::sin(half_roll) * std::cos(half_pitch) * std::sin(half_yaw);
  double z = std::cos(half_roll) * std::cos(half_pitch) * std::sin(half_yaw) - std::sin(half_roll) * std::sin(half_pitch) * std::cos(half_yaw);

  target_pose.orientation.x = x; 
  target_pose.orientation.y = y;
  target_pose.orientation.z = z;
  target_pose.orientation.w = w;  
 
  move_group2.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (move_group2.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group2.execute(plan);

  //RCLCPP_WARN(
    //node_->get_logger(),
    //"Task 1 is not implemented in cw2_team_11. Latest cloud: seq=%llu frame='%s' points=%zu",
    //static_cast<unsigned long long>(sequence),
    //frame_id.c_str(),
    //point_count);
}

void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  auto L = node_->get_logger();
  RCLCPP_INFO(L, "Starting Task 2");

  static const std::string planning_group = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group2(node_, planning_group);

  move_group2.setPlanningTime(5.0);
  move_group2.setMaxVelocityScalingFactor(0.2);
  move_group2.setMaxAccelerationScalingFactor(0.2);
  
  geometry_msgs::msg::Pose target_pose; // Declare target_pose here so it can be reused for all three moves (we will just update the position each time)

  double roll = M_PI; // 180 degrees
  double pitch = 0.0;
  double yaw = -M_PI / 4.0; // -45 degrees

  double half_roll = roll / 2.0;
  double half_pitch = pitch / 2.0;
  double half_yaw = yaw / 2.0;


  double w = std::cos(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) + std::sin(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
  double x = std::sin(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) - std::cos(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
  double y = std::cos(half_roll) * std::sin(half_pitch) * std::cos(half_yaw) + std::sin(half_roll) * std::cos(half_pitch) * std::sin(half_yaw);
  double z = std::cos(half_roll) * std::cos(half_pitch) * std::sin(half_yaw) - std::sin(half_roll) * std::sin(half_pitch) * std::cos(half_yaw);
 
  target_pose.orientation.x = x; 
  target_pose.orientation.y = y;
  target_pose.orientation.z = z;
  target_pose.orientation.w = w;  

    //Reference object 1
  
  target_pose.position.x = request->ref_object_points[0].point.x;
  target_pose.position.y = request->ref_object_points[0].point.y;
  target_pose.position.z = request->ref_object_points[0].point.z + 0.5;
  move_group2.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan1;

  bool success = (move_group2.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group2.execute(plan1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  
  //Reference object 2
  target_pose.position.x = request->ref_object_points[1].point.x;
  target_pose.position.y = request->ref_object_points[1].point.y;
  target_pose.position.z = request->ref_object_points[1].point.z + 0.5;

  move_group2.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan2;

  success = (move_group2.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group2.execute(plan2);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  //Mystery object
  target_pose.position.x = request->mystery_object_point.point.x;
  target_pose.position.y = request->mystery_object_point.point.y;
  target_pose.position.z = request->mystery_object_point.point.z + 0.5;

  move_group2.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan3;

  success = (move_group2.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group2.execute(plan3);
  
}


void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  (void)request;
  auto L = node_->get_logger();
  RCLCPP_INFO(L, "Starting Task 3: Explicit Perimeter Sweep...");

  static const std::string planning_group = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group3(node_, planning_group);

  move_group3.setPlanningTime(10.0); //maybe chabge that to 5 seconds
  move_group3.setMaxVelocityScalingFactor(0.2);
  move_group3.setMaxAccelerationScalingFactor(0.2);
  
  geometry_msgs::msg::Pose target_pose; // Declare target_pose here so it can be reused for all three moves (we will just update the position each time)

  double roll = M_PI; // 180 degrees
  double pitch = 0.0;
  double yaw = -M_PI / 4.0; // -45 degrees

  double half_roll = roll / 2.0;
  double half_pitch = pitch / 2.0;
  double half_yaw = yaw / 2.0;


  double w = std::cos(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) + std::sin(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
  double x = std::sin(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) - std::cos(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
  double y = std::cos(half_roll) * std::sin(half_pitch) * std::cos(half_yaw) + std::sin(half_roll) * std::cos(half_pitch) * std::sin(half_yaw);
  double z = std::cos(half_roll) * std::cos(half_pitch) * std::sin(half_yaw) - std::sin(half_roll) * std::sin(half_pitch) * std::cos(half_yaw);
 
  target_pose.orientation.x = x; 
  target_pose.orientation.y = y;
  target_pose.orientation.z = z;
  target_pose.orientation.w = w; 

//Move 1

target_pose.position.x = -0.45;
target_pose.position.y = 0.35;
target_pose.position.z = 0.65;

move_group3.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan1;

bool success = (move_group3.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
move_group3.execute(plan1);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//Move 1.2
target_pose.position.x = 0;
target_pose.position.y = 0.35;
target_pose.position.z = 0.65;

move_group3.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan2;

success = (move_group3.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
move_group3.execute(plan2);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//Move 2
target_pose.position.x = 0.45;
target_pose.position.y = 0.35;
target_pose.position.z = 0.65;

move_group3.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan3;

success = (move_group3.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
move_group3.execute(plan3);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//Move 2.2

target_pose.position.x = 0.45;
target_pose.position.y = 0;
target_pose.position.z = 0.65;

move_group3.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan4;

success = (move_group3.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS);
move_group3.execute(plan4);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//Move 3

target_pose.position.x = 0.45;
target_pose.position.y = -0.35;
target_pose.position.z = 0.65;

move_group3.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan5;

success = (move_group3.plan(plan5) == moveit::core::MoveItErrorCode::SUCCESS);
move_group3.execute(plan5);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//Move 3.2

target_pose.position.x = 0;
target_pose.position.y = -0.35;
target_pose.position.z = 0.65;

move_group3.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan6;

success = (move_group3.plan(plan6) == moveit::core::MoveItErrorCode::SUCCESS);
move_group3.execute(plan6);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//Move 4

target_pose.position.x = -0.45;
target_pose.position.y = -0.35;
target_pose.position.z = 0.65;

move_group3.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan7;

success = (move_group3.plan(plan7) == moveit::core::MoveItErrorCode::SUCCESS);
move_group3.execute(plan7);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//Move 4.2

target_pose.position.x = -0.45;
target_pose.position.y = 0;
target_pose.position.z = 0.65;

move_group3.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan8;

success = (move_group3.plan(plan8) == moveit::core::MoveItErrorCode::SUCCESS);
move_group3.execute(plan8);
std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

//PCL FUNCTIONS


void cw2::applyVoxelGrid(double g_leaf_size)
{
  
  PointCPtr output_cloud(new PointC);

  g_vx.setInputCloud(g_cloud_filtered);
  g_vx.setLeafSize(g_leaf_size, g_leaf_size, g_leaf_size);
  g_vx.filter(*output_cloud);

  g_cloud_filtered.swap(output_cloud);

}

void cw2::applyPassthrough(double g_pass_min, double g_pass_max, std::string g_pass_axis)
{
  
  PointCPtr output_cloud(new PointC);

  g_pt.setInputCloud(g_cloud_filtered);
  g_pt.setFilterFieldName(g_pass_axis);
  g_pt.setFilterLimits(g_pass_min, g_pass_max);
  g_pt.filter(*output_cloud);
  g_cloud_filtered.swap(output_cloud);


}

void cw2::applyOutlierRemoval(int g_outlier_mean_k, double g_outlier_stddev)
{
  
  PointCPtr output_cloud(new PointC);

  g_sor.setInputCloud(g_cloud_filtered);
  g_sor.setMeanK(g_outlier_mean_k);
  g_sor.setStddevMulThresh(g_outlier_stddev);
  g_sor.filter(*output_cloud);
  g_cloud_filtered.swap(output_cloud);

}

void cw2::findNormals(int g_normal_k)
{
  g_ne.setInputCloud(g_cloud_filtered);
  g_ne.setSearchMethod(g_tree_ptr);
  g_ne.setKSearch(g_normal_k);
  g_ne.compute(*g_cloud_normals);
}

void cw2::segmentationPipeline(double g_plane_normal_dist_weight, int g_plane_max_iterations, double g_plane_distance)
{
  // TODO(student-9): Implement normal-plane segmentation.

  //Configure model
  g_seg.setOptimizeCoefficients(true);
  g_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight(g_plane_normal_dist_weight);
  g_seg.setMethodType(pcl::SAC_RANSAC);
  g_seg.setMaxIterations(g_plane_max_iterations);
  g_seg.setDistanceThreshold(g_plane_distance);

  //Set cloud to segment
  g_seg.setInputCloud(g_cloud_filtered);
  g_seg.setInputNormals(g_cloud_normals);

  //segment
  g_seg.segment(*g_inliers_plane, *g_coeff_plane);

  //extract point cloud that is the plane with inliers
  g_extract_pc.setInputCloud(g_cloud_filtered);
  g_extract_pc.setIndices(g_inliers_plane);
  g_extract_pc.setNegative(false);
  g_extract_pc.filter(*g_cloud_plane);

  //extract point cloud that is NOT the plane (outliers). Store in filtered 2
  g_extract_pc.setNegative(true);
  g_extract_pc.filter(*g_cloud_segmented_plane);

  //remove normals from the normal cloud that are in the plane
  //normals2 is just normals of non-plane items
  g_extract_normals.setNegative(true);
  g_extract_normals.setInputCloud(g_cloud_normals);
  g_extract_normals.setIndices(g_inliers_plane);
  g_extract_normals.filter(*g_cloud_segmented_normals);
}

std::vector<PointCPtr> cw2::extractEuclideanClusters(double clusterTolerance, int minClusterSize, int maxClusterSize)
{
  g_tree_ptr_euclidean->setInputCloud(g_cloud_segmented_plane);
  //Configure clustering
  std::vector<pcl::PointIndices> cluster_indices;
  g_extract_euclidean.setClusterTolerance(clusterTolerance); // 2cm
  g_extract_euclidean.setMinClusterSize(minClusterSize);
  g_extract_euclidean.setMaxClusterSize(maxClusterSize);
  g_extract_euclidean.setSearchMethod(g_tree_ptr_euclidean);
  g_extract_euclidean.setInputCloud(g_cloud_segmented_plane);

  //Extract clusters
  g_extract_euclidean.extract(cluster_indices);
  
  int num_cluster = 0;

  std::vector<PointCPtr> all_clouds;

  for (const auto& cluster : cluster_indices)
  {
    PointCPtr cloud_cluster(new PointC);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*g_cloud_segmented_plane)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;


    if (cloud_cluster->size() == 0)
    {
      continue;
    }

    if (num_cluster < 6)
    {
    
      // For testing only
      std_msgs::msg::Header header;
      header.frame_id = "color";
      header.stamp = latest_cloud_msg_->header.stamp;
      pubFilteredPCMsg(g_pub_clusters[num_cluster], *cloud_cluster, header);

    }
    //for testing only

    num_cluster = num_cluster + 1;

    all_clouds.push_back(cloud_cluster);
  }
  
  return all_clouds;

}


Eigen::Vector3f cw2::getCentroid(PointC &in_cloud_ptr)
{
  
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(in_cloud_ptr, centroid);

  return centroid.head<3>();  // drops the homogeneous w component
}

cw2::SHAPE cw2::classifyShape(PointC &in_cloud_ptr)
{

  //Discrepancy here.


  cw2::SHAPE shape;
  shape.type = cw2::SHAPE_TYPE::CROSS;
  shape.size = cw2::SHAPE_SIZE::MM_40;

  return shape;

}

//publish ros message for debug 
void cw2::pubFilteredPCMsg(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pc_pub,
    PointC &pc,
    const std_msgs::msg::Header &header)
{

  // publish type
  sensor_msgs::msg::PointCloud2 output;

  //pass input cloud, output PointCloud2 by reference
  pcl::toROSMsg(pc, output);
  output.header = header;
  pc_pub->publish(output);

}

//debug helper
void cw2::processCloud()
{
  if (!latest_cloud_msg_) {
    RCLCPP_WARN(node_->get_logger(), "reprocessCloud: no cloud yet, skipping.");
    return;
  }
  std_msgs::msg::Header header;
  header.frame_id = "color";
  header.stamp = latest_cloud_msg_->header.stamp;

  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered, header);
  applyPassthrough(pcl_pass_min_, pcl_pass_max_, pcl_pass_axis_);
  pubFilteredPCMsg(g_pub_passthrough, *g_cloud_filtered, header);
  applyOutlierRemoval(pcl_outlier_mean_k_, pcl_outlier_stddev_);
  pubFilteredPCMsg(g_pub_outlier, *g_cloud_filtered, header);
  findNormals(pcl_normal_k_);
  segmentationPipeline(pcl_plane_normal_weight_, pcl_plane_max_iterations_, pcl_plane_distance_);
  pubFilteredPCMsg(g_pub_plane, *g_cloud_segmented_plane, header);
  extractEuclideanClusters(pcl_cluster_tolerance_, pcl_cluster_min_size_, pcl_cluster_max_size_);

}

//get color of a point cloud
std::string cw2::colorOfPointCloud(PointC &in_cloud_ptr, float threshold)
{

  float r = 0;
  float g = 0;
  float b = 0;

  //average all point colors in the cloud

  for (const auto & pt : in_cloud_ptr.points)
  {
    r = r + (pt.r / 255.0);
    g = g + (pt.g / 255.0);
    b = b + (pt.b / 255.0);
  }

  r = r / in_cloud_ptr.size();
  g = g / in_cloud_ptr.size();
  b = b / in_cloud_ptr.size();

  float min_dist = 1000;
  int min_color_idx;

  RCLCPP_INFO(node_->get_logger(), "num_pts = %d, r=%.3f g=%.3f b=%.3f", static_cast<int>(in_cloud_ptr.size()), r, g, b);


  // find closest color
  for (size_t i = 0; i < num_colors; i++)
  {

    std::array<float, 3> color = colors[i];
    //distance between average and saved color
    float dist = std::sqrt(std::pow(r - color[0], 2) + std::pow(g - color[1], 2) + std::pow(b - color[2], 2));


    if (dist < min_dist)
    {
      min_dist = dist;
      min_color_idx = i;
    }

  }
  //check to see if closest color is in range
  if (min_dist < threshold)
  {
    return color_names[min_color_idx];
  }
  else
  {
    return no_color;
  }

}

void cw2::filteringPipeline()
{
  // rosTopicToCloud(latest_cloud_msg_);
  // applyVoxelGrid(0.05);
  applyPassthrough(-0.31, 0.18, "y");
  applyOutlierRemoval(20, 1.0);
  findNormals(50);
  segmentationPipeline(0.1, 100, 0.03);
}

//convert local coords to world coords
Eigen::Vector3f cw2::toWorldFrame(Eigen::Vector3f local_point)
{
  geometry_msgs::msg::PointStamped local, world;
  local.header.frame_id = g_input_pc_frame_id;
  local.header.stamp = latest_cloud_msg_->header.stamp;
  local.point.x = local_point.x();
  local.point.y = local_point.y();
  local.point.z = local_point.z();

  try {
    tf_buffer_.transform(local, world, "panda_link0");
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Transform failed: %s", ex.what());
    return local_point;
  }

  return Eigen::Vector3f(world.point.x, world.point.y, world.point.z);
}