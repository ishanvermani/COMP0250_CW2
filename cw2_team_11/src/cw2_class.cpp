/* COMP0250 Coursework 2 - Team 11
 *
 * Task 1: Pick and place a nought or cross shape into a basket.
 * Task 2: Shape detection (stub).
 * Task 3: Planning and execution (stub).
 */

#include <cw2_class.h>

#include <utility>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h> 

static constexpr double FTIP = 0.105;  // link8 to fingertip offset

static geometry_msgs::msg::Pose td_pose(double x, double y, double z, double yaw = -M_PI/4.0)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; 
  p.position.y = y; 
  p.position.z = z;

  
  double roll = M_PI; 
  double pitch = 0.0;
  
  //double yaw = -M_PI/4.0;

  double half_roll = roll / 2.0;
  double half_pitch = pitch / 2.0;
  double half_yaw = yaw / 2.0;

  // Manual explicit Euler to Quaternion calculation
  double qw = std::cos(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) + std::sin(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
  double qx = std::sin(half_roll) * std::cos(half_pitch) * std::cos(half_yaw) - std::cos(half_roll) * std::sin(half_pitch) * std::sin(half_yaw);
  double qy = std::cos(half_roll) * std::sin(half_pitch) * std::cos(half_yaw) + std::sin(half_roll) * std::cos(half_pitch) * std::sin(half_yaw);
  double qz = std::cos(half_roll) * std::cos(half_pitch) * std::sin(half_yaw) - std::sin(half_roll) * std::sin(half_pitch) * std::cos(half_yaw);

  p.orientation.x = qx;
  p.orientation.y = qy;
  p.orientation.z = qz;
  p.orientation.w = qw;
  
  return p;
}

static inline double ft2l8(double z) { return z + FTIP; }

static bool joint_move(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &m,
  const geometry_msgs::msg::Pose &t, const rclcpp::Logger &l,
  const std::string &d, int n = 5)
{
  m->setPoseTarget(t);
  for (int a = 1; a <= n; ++a) {
    moveit::planning_interface::MoveGroupInterface::Plan p;
    if (m->plan(p) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(l, "%s: plan %d/%d", d.c_str(), a, n); continue;
    }
    if (m->execute(p) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(l, "%s: OK", d.c_str()); return true;
    }
  }
  RCLCPP_ERROR(l, "%s: FAIL", d.c_str()); return false;
}

static bool cart_move(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &m,
  const geometry_msgs::msg::Pose &t, const rclcpp::Logger &l,
  const std::string &d)
{
  std::vector<geometry_msgs::msg::Pose> w = {t};
  moveit_msgs::msg::RobotTrajectory tr;
  double f = m->computeCartesianPath(w, 0.005, 0.0, tr);
  if (f >= 0.90 && m->execute(tr) == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(l, "%s: Cart OK (%.0f%%)", d.c_str(), f * 100); return true;
  }
  RCLCPP_WARN(l, "%s: Cart %.0f%%, joint fallback", d.c_str(), f * 100);
  return joint_move(m, t, l, d);
}

static bool open_gripper(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &h,
  const rclcpp::Logger &l)
{
  h->setNamedTarget("open");
  moveit::planning_interface::MoveGroupInterface::Plan p;

  if (h->plan(p) != moveit::core::MoveItErrorCode::SUCCESS) return false;
  if (h->execute(p) != moveit::core::MoveItErrorCode::SUCCESS) return false;
  RCLCPP_INFO(l, "Gripper OPEN"); return true;
}

// Grip with force scaled to shape size.
// cell_size_mm: arm width of the shape (20, 30, or 40).
// Target = (arm_half - 5mm) so there's always 5mm of squeeze.
//   x=40 → target=0.015, squeeze=5mm ✓
//   x=30 → target=0.010, squeeze=5mm ✓
//   x=20 → target=0.005, squeeze=5mm ✓
// Without scaling, x=20 shapes get PUSHED OUT (target > shape width)
static void strong_grip(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &h,
  const rclcpp::Logger &l,
  int cell_size_mm = 40)
{
  // Target = arm_half - 10mm squeeze. Maximum force for small shapes.
  //   x=40 → target=0.010, squeeze=10mm
  //   x=30 → target=0.005, squeeze=10mm
  //   x=20 → target=0.001, squeeze=9mm (nearly fully closed)
  double arm_half_m = (cell_size_mm / 2.0) / 1000.0;
  double target = std::max(0.001, arm_half_m - 0.010);
  
  // FULL SPEED grip — slow speed (0.05) causes controller timeout
  // before fingers close enough on small shapes. CW1 used 1.0.
  h->setMaxVelocityScalingFactor(0.1);
  h->setMaxAccelerationScalingFactor(0.1);
  RCLCPP_INFO(l, "  GRIP (size=%dmm, target=%.4f, squeeze=8mm)", cell_size_mm, target);
  
  h->setJointValueTarget("panda_finger_joint1", target);
  h->setJointValueTarget("panda_finger_joint2", target);
  
  moveit::planning_interface::MoveGroupInterface::Plan p;
  if (h->plan(p) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(l, "  Grip plan failed"); return;
  }
  h->execute(p);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  RCLCPP_INFO(l, "  Grip applied");
}

static bool go_home(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &m,
  const rclcpp::Logger &l)
{
  m->setNamedTarget("ready");
  for (int a = 1; a <= 5; ++a) {
    moveit::planning_interface::MoveGroupInterface::Plan p;
    if (m->plan(p) != moveit::core::MoveItErrorCode::SUCCESS) continue;
    if (m->execute(p) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(l, "Home OK"); return true;
    }
  }
  RCLCPP_ERROR(l, "Home FAIL"); return false;
}



cw2::cw2(const rclcpp::Node::SharedPtr &node)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_)
{

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

  t1_service_ = node_->create_service<cw2_world_spawner::srv::Task1Service>(
    "/task1_start",
    std::bind(&cw2::t1_callback, this, std::placeholders::_1, std::placeholders::_2));
  t2_service_ = node_->create_service<cw2_world_spawner::srv::Task2Service>(
    "/task2_start",
    std::bind(&cw2::t2_callback, this, std::placeholders::_1, std::placeholders::_2));
  t3_service_ = node_->create_service<cw2_world_spawner::srv::Task3Service>(
    "/task3_start",
    std::bind(&cw2::t3_callback, this, std::placeholders::_1, std::placeholders::_2));

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

  g_pub_pose = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/cw2_pose", 1);

  g_pub_clusters = {g_pub_cluster1, g_pub_cluster2, g_pub_cluster3, g_pub_cluster4, g_pub_cluster5, g_pub_cluster6};


  color_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_,
    pointcloud_qos,
    std::bind(&cw2::cloud_callback, this, std::placeholders::_1),
    pointcloud_sub_options);

  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "panda_arm");
  hand_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand");

  
  arm_group_->setPlanningTime(10.0);
  arm_group_->setNumPlanningAttempts(10);
  arm_group_->setMaxVelocityScalingFactor(0.5);
  arm_group_->setMaxAccelerationScalingFactor(0.5);

  RCLCPP_INFO(node_->get_logger(), "CW2 Team 11 initialised");
}



void cw2::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // pcl::PCLPointCloud2 pcl_cloud;
  // pcl_conversions::toPCL(*msg, pcl_cloud);

  // PointCPtr latest_cloud(new PointC);
  // pcl::fromPCLPointCloud2(pcl_cloud, *latest_cloud);

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  // g_input_pc_frame_id_ = msg->header.frame_id;
  // g_cloud_ptr = std::move(latest_cloud);
  // ++g_cloud_sequence_;

  latest_cloud_msg_ = msg;

}

///////////////////////////////////////////////////////////////////////////////
// Reusable pick-and-place for ANY shape size (x = 20, 30, or 40mm)
// Offsets and grip force scale automatically with cell_size_mm.
///////////////////////////////////////////////////////////////////////////////

bool cw2::pick_and_place_shape(
  double ox, double oy,
  double bx, double by, double bz,
  const cw2::SHAPE &shape)
{
  auto L = node_->get_logger();
  double cell_m = static_cast<float>(shape.size) / 1000.0;

  // Scale grasp offset with cell size.
  // nought: 2.0 * cell = exact center of the side wall. 
  //   The side is 1 cell thick at 2*cell from center.
  //   x=40 → 80mm, x=30 → 60mm, x=20 → 40mm (all centered on wall)
  // cross:  1.0 * cell = first arm cell from center (secure, away from intersection)
  double px = ox, py = oy;
  if (shape.type == cw2::SHAPE_TYPE::NOUGHT) {
    py = oy - 2.0 * cell_m;
  } else {
    px = ox + 1.0 * cell_m;
  }

  // Drop offsets (small, just to clear basket rim)
  double dx = bx, dy = by;
  if (shape.type == cw2::SHAPE_TYPE::NOUGHT) {
    dy = by - 1.5 * cell_m;
  } else {
    dx = bx + 0.5 * cell_m;
  }

  RCLCPP_INFO(L, "pick_and_place: shape=%d size=%dmm grasp=(%.3f,%.3f) drop=(%.3f,%.3f)",
              static_cast<int>(shape.type), static_cast<int>(shape.size), px, py, dx, dy);

  // Heights — shape is ALWAYS 40mm tall regardless of cell size
  double grip_z   = 0.020 + 0.15;
  double safe_z   = 0.020 + 0.65;
  double basket_z = bz + 0.35;

  auto fail = [&]() { open_gripper(hand_group_, L); go_home(arm_group_, L); };

  go_home(arm_group_, L);

  // Move A: high above shape
  RCLCPP_INFO(L, "Move A: Above shape");
  if (!joint_move(arm_group_, td_pose(px, py, safe_z), L, "Above shape")) { fail(); return false; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  if (!open_gripper(hand_group_, L)) { fail(); return false; }

  // double shape_yaw_rad = t1_shape.yaw * (M_PI / 180.0);

  // Move B: descend to grip
  RCLCPP_INFO(L, "Move B: Descend");
  arm_group_->setMaxVelocityScalingFactor(0.1);
  arm_group_->setMaxAccelerationScalingFactor(0.1);
  if (!cart_move(arm_group_, td_pose(px, py, grip_z), L, "Descend")) { fail(); return false; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Move C: grip (scaled to shape size!)
  RCLCPP_INFO(L, "Move C: Grip (size=%dmm)", static_cast<int>(shape.size));
  strong_grip(hand_group_, L, static_cast<int>(shape.size));

 
  // Move D: lift
  RCLCPP_INFO(L, "Move D: Lift");
  arm_group_->setMaxVelocityScalingFactor(0.5);
  arm_group_->setMaxAccelerationScalingFactor(0.5);
  if (!cart_move(arm_group_, td_pose(px, py, safe_z), L, "Lift")) { fail(); return false; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Move E: above basket
  RCLCPP_INFO(L, "Move E: Above basket");
  if (!joint_move(arm_group_, td_pose(dx, dy, safe_z), L, "Above basket")) { fail(); return false; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Move F: lower into basket
  RCLCPP_INFO(L, "Move F: Lower");
  if (!cart_move(arm_group_, td_pose(dx, dy, basket_z), L, "Lower")) { fail(); return false; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Move G: release
  RCLCPP_INFO(L, "Move G: Release");
  moveit::planning_interface::MoveGroupInterface hand_group(node_, "hand");
  hand_group.setMaxVelocityScalingFactor(1.0);
  hand_group.setMaxAccelerationScalingFactor(1.0);
  hand_group.setNamedTarget("open");
  moveit::planning_interface::MoveGroupInterface::Plan hp;
  hand_group.plan(hp);
  hand_group.execute(hp);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Move H: retreat
  RCLCPP_INFO(L, "Move H: Retreat");
  if (!joint_move(arm_group_, td_pose(dx, dy, safe_z), L, "Retreat")) { fail(); return false; }
  go_home(arm_group_, L);

  RCLCPP_INFO(L, "pick_and_place: DONE");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Task 1: Pick and Place
///////////////////////////////////////////////////////////////////////////////

void cw2::t1_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> response)
{
  (void)response;
  auto L = node_->get_logger();

  // Log raw input frames
  RCLCPP_INFO(L, "=== Task 1 ===");
  RCLCPP_INFO(L, "Object frame: '%s'  Goal frame: '%s'",
              request->object_point.header.frame_id.c_str(),
              request->goal_point.header.frame_id.c_str());

  
  geometry_msgs::msg::PointStamped obj_world, goal_world;
  const std::string target_frame = "panda_link0";
  try {
    obj_world = tf_buffer_.transform(request->object_point, target_frame,
                                     tf2::durationFromSec(2.0));
    goal_world = tf_buffer_.transform(request->goal_point, target_frame,
                                      tf2::durationFromSec(2.0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(L, "TF transform failed: %s", ex.what());
    return;
  }

  const double ox = obj_world.point.x;
  const double oy = obj_world.point.y;
  const double oz = obj_world.point.z;
  const double gx = goal_world.point.x;
  const double gy = goal_world.point.y;
  const double gz = goal_world.point.z;
  const std::string shape = request->shape_type;

  RCLCPP_INFO(L, "Shape: %s at (%.4f, %.4f, %.4f) [world]", shape.c_str(), ox, oy, oz);
  RCLCPP_INFO(L, "Basket at (%.4f, %.4f, %.4f) [world]", gx, gy, gz);

  // double gx_pick, gy_pick;
  // double gx_drop, gy_drop;

  

  // const double shape_centroid_z = 0.020;           
  // const double grasp_l8     = ft2l8(shape_centroid_z + 0.015); 
  // const double pre_grasp_l8 = grasp_l8 + 0.085;    
  // const double transit_l8   = 0.40;                 
  // const double release_l8   = ft2l8(gz + 0.10);  

  

  // RCLCPP_INFO(L, "Heights: grasp_l8=%.4f pre_grasp=%.4f transit=%.4f release=%.4f",
  //             grasp_l8, pre_grasp_l8, transit_l8, release_l8);

  double grip_z = request->object_point.point.z + 0.15;
  double basket_z = request->goal_point.point.z + 0.35;
  double safe_z = request->object_point.point.z + 0.65;

  moveit::planning_interface::MoveGroupInterface hand_group(node_, "hand");
  hand_group.setMaxVelocityScalingFactor(1.0);
  hand_group.setMaxAccelerationScalingFactor(1.0);

  auto fail = [&]() {
    open_gripper(hand_group_, L);
    go_home(arm_group_, L);
  };

  
  go_home(arm_group_, L);
  // Move 1
  RCLCPP_INFO(L, "Move A: Moving high above shape");
  if (!joint_move(arm_group_, td_pose(ox, oy, safe_z), L, "Move A")) { fail(); return; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  if (!open_gripper(hand_group_, L)) { fail(); return; }

  // Determine shape yaw

  std::vector<PointCPtr> clusters = findClusters();

  cw2::SHAPE t1_shape;
  
  for (size_t i = 0; i < clusters.size(); i++)
  {
    RCLCPP_INFO(node_->get_logger(), "Classifyting Cluster %ld", i);
    t1_shape = classifyShape(clusters[i]);
    if(t1_shape.type == cw2::SHAPE_TYPE::UNKNOWN || t1_shape.size == cw2::SHAPE_SIZE::UNKNOWN)
    {
      continue;
    }
  }

  //
  // USE t1_shape.yaw FOR YAW!!
  //

  double shape_yaw_rad = t1_shape.yaw * (M_PI / 180.0);
  double gripper_yaw = shape_yaw_rad - M_PI/4.0;

  double gx_pick = ox;
  double gy_pick = oy;
  double gx_drop = gx;
  double gy_drop = gy;

  // if (shape == "nought") {
  //   RCLCPP_INFO(L, "Applying nought offsets");
  //   gx_pick = ox + 0.08 * std::sin(shape_yaw_rad);
  //   gy_pick = oy - 0.08 * std::cos(shape_yaw_rad);
  //   gx_drop = gx;
  //   gy_drop = gy - 0.05;
  // } else {
  //   RCLCPP_INFO(L, "Applying cross offsets");
  //   gx_pick = ox + 0.05 * std::cos(shape_yaw_rad);
  //   gy_pick = oy + 0.05 * std::sin(shape_yaw_rad);
  //   gx_drop = gx + 0.02;
  //   gy_drop = gy;
  // }

  if (t1_shape.type == cw2::SHAPE_TYPE::NOUGHT) {
    if (t1_shape.size == cw2::SHAPE_SIZE::MM_40){
    RCLCPP_INFO(L, "Applying nought offsets");
    gx_pick = ox + 0.08 * std::sin(shape_yaw_rad);
    gy_pick = oy - 0.08 * std::cos(shape_yaw_rad);
    gx_drop = gx;
    gy_drop = gy - 0.05;
    } else if (t1_shape.size == cw2::SHAPE_SIZE::MM_30){
    RCLCPP_INFO(L, "Applying nought offsets");
    gx_pick = ox + 0.06 * std::sin(shape_yaw_rad);
    gy_pick = oy - 0.06 * std::cos(shape_yaw_rad);
    gx_drop = gx;
    gy_drop = gy - 0.05;
    } else if (t1_shape.size == cw2::SHAPE_SIZE::MM_20){
      RCLCPP_INFO(L, "Applying nought offsets");
    gx_pick = ox + 0.05 * std::sin(shape_yaw_rad);
    gy_pick = oy - 0.05 * std::cos(shape_yaw_rad);
    gx_drop = gx;
    gy_drop = gy - 0.05;
    }
  } else {
    if (t1_shape.size == cw2::SHAPE_SIZE::MM_40){
    RCLCPP_INFO(L, "Applying cross offsets");
    gx_pick = ox + 0.05 * std::cos(shape_yaw_rad);
    gy_pick = oy + 0.05 * std::sin(shape_yaw_rad);
    gx_drop = gx + 0.02;
    gy_drop = gy;
    } else if (t1_shape.size == cw2::SHAPE_SIZE::MM_30){
    RCLCPP_INFO(L, "Applying cross offsets");
    gx_pick = ox + 0.04 * std::cos(shape_yaw_rad);
    gy_pick = oy + 0.04 * std::sin(shape_yaw_rad);
    gx_drop = gx + 0.02;
    gy_drop = gy;
    } else {
      RCLCPP_INFO(L, "Applying cross offsets");
    gx_pick = ox + 0.037 * std::cos(shape_yaw_rad);
    gy_pick = oy + 0.037 * std::sin(shape_yaw_rad);
    gx_drop = gx + 0.02;
    gy_drop = gy;
    }

    
  }
  

  // Move 2
  RCLCPP_INFO(L, "Move B: Descending to grip");
  arm_group_->setMaxVelocityScalingFactor(0.1);
  arm_group_->setMaxAccelerationScalingFactor(0.1);
  if (!cart_move(arm_group_, td_pose(gx_pick, gy_pick, grip_z, gripper_yaw), L, "Move B")) { fail(); return; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  //Move 3

  //OLD GRIP//

  if (t1_shape.size == cw2::SHAPE_SIZE::MM_20 and t1_shape.type == cw2::SHAPE_TYPE::CROSS){
  RCLCPP_INFO(L, "Move C: Closing gripper. I WANT THE BIG BUNNY");
  strong_grip(hand_group_, L, 40);
}
  else {

   //I think this grip works faster
  RCLCPP_INFO(L, "Closing gripper. TOUT TOUT TOUT. I WANT THE BIG BUNNY");
  hand_group.setJointValueTarget("panda_finger_joint1", 0.020); 
  moveit::planning_interface::MoveGroupInterface::Plan close_plan;
  hand_group.plan(close_plan) == moveit::core::MoveItErrorCode::SUCCESS;
  hand_group.execute(close_plan);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

  //Move 4
  RCLCPP_INFO(L, "Move D: Lifting object to safe height");
  arm_group_->setMaxVelocityScalingFactor(0.5);
  arm_group_->setMaxAccelerationScalingFactor(0.5);
  if (!cart_move(arm_group_, td_pose(gx_pick, gy_pick, safe_z, gripper_yaw), L, "Move D")) { fail(); return; }
  //std::this_thread::sleep_for(std::chrono::milliseconds(200));

  //Move 5
  RCLCPP_INFO(L, "Move E: Translating over obstacles to basket");
  if (!joint_move(arm_group_, td_pose(gx_drop, gy_drop, safe_z), L, "Move E")) { fail(); return; }
  //std::this_thread::sleep_for(std::chrono::milliseconds(200));

  //Move 6
  RCLCPP_INFO(L, "Move F: Descending into basket");
  if (!cart_move(arm_group_, td_pose(gx_drop, gy_drop, basket_z), L, "Move F")) { fail(); return; }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  

  // //Move 7

  // if (!open_gripper(hand_group_, L)) { fail(); return; }

  //OLD RELEASE//
  // RCLCPP_INFO(L, "Move G: KOBE! Releasing shape");
  // moveit::planning_interface::MoveGroupInterface hand_group(node_, "hand");
  // hand_group.setMaxVelocityScalingFactor(1.0);
  // hand_group.setMaxAccelerationScalingFactor(1.0);
  // moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
  // hand_group.setNamedTarget("open");
  // bool success = (hand_group.plan(hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // hand_group.execute(hand_plan);
  // std::this_thread::sleep_for(std::chrono::milliseconds(200));


  //I think this release works faster
  RCLCPP_INFO(L, "KOBE!");
  hand_group.setNamedTarget("open");
  moveit::planning_interface::MoveGroupInterface::Plan final_open_plan;
  hand_group.plan(final_open_plan) == moveit::core::MoveItErrorCode::SUCCESS;
  hand_group.execute(final_open_plan);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));


  //Move 8
  RCLCPP_INFO(L, "Move F: Ascending above basket");
  if (!cart_move(arm_group_, td_pose(gx_drop, gy_drop, safe_z), L, "Move F")) { fail(); return; }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  //Move 9
  RCLCPP_INFO(L, "Returning to 'ready' home position...");
  go_home(arm_group_, L);
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); 


  RCLCPP_INFO(L, "=== Task 1 complete ===");
}

///////////////////////////////////////////////////////////////////////////////
// Task 2: Shape Detection (stub)
///////////////////////////////////////////////////////////////////////////////

void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  auto L = node_->get_logger();
  RCLCPP_INFO(L, "Starting Task 2");

  (void)request;
  response->mystery_object_num = -1;

  cw2::SHAPE reference_shape_1 = {cw2::SHAPE_TYPE::UNKNOWN, cw2::SHAPE_SIZE::UNKNOWN, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0};
  cw2::SHAPE reference_shape_2 = {cw2::SHAPE_TYPE::UNKNOWN, cw2::SHAPE_SIZE::UNKNOWN, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0};
  cw2::SHAPE mystery_shape = {cw2::SHAPE_TYPE::UNKNOWN, cw2::SHAPE_SIZE::UNKNOWN, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0};
  std::vector<PointCPtr> clusters;

  
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

  if(move_group2.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group2.execute(plan1);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    clusters = findClusters();
    
    for (size_t i = 0; i < clusters.size(); i++)
    {
      RCLCPP_INFO(node_->get_logger(), "Classifyting Cluster %ld", i);

      reference_shape_1 = classifyShape(clusters[i]);
      
      if (reference_shape_1.type != cw2::SHAPE_TYPE::UNKNOWN && reference_shape_1.size != cw2::SHAPE_SIZE::UNKNOWN) 
      {
        break; 
      }
    }

  }
  else
  {
    RCLCPP_WARN(L, "Error moving to reference shape 1");
    return;
  }
  

  
  //Reference object 2
  target_pose.position.x = request->ref_object_points[1].point.x;
  target_pose.position.y = request->ref_object_points[1].point.y;
  target_pose.position.z = request->ref_object_points[1].point.z + 0.5;

  move_group2.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan2;

  if(move_group2.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group2.execute(plan2);
 
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  

    clusters = findClusters();
    
    for (size_t i = 0; i < clusters.size(); i++)
    {
      RCLCPP_INFO(node_->get_logger(), "Classifyting Cluster %ld", i);

      reference_shape_2 = classifyShape(clusters[i]);
      
      if (reference_shape_2.type != cw2::SHAPE_TYPE::UNKNOWN && reference_shape_2.size != cw2::SHAPE_SIZE::UNKNOWN) 
      {
        break; 
      }
    }

  }
  else
  {
    RCLCPP_WARN(L, "Error moving to reference shape 2");
    return;
  }


  //Mystery object
  target_pose.position.x = request->mystery_object_point.point.x;
  target_pose.position.y = request->mystery_object_point.point.y;
  target_pose.position.z = request->mystery_object_point.point.z + 0.5;

  move_group2.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan3;

  if(move_group2.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group2.execute(plan3);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    clusters = findClusters();
    
    for (size_t i = 0; i < clusters.size(); i++)
    {
      RCLCPP_INFO(node_->get_logger(), "Classifyting Cluster %ld", i);

      mystery_shape = classifyShape(clusters[i]);
      
      if (mystery_shape.type != cw2::SHAPE_TYPE::UNKNOWN && mystery_shape.size != cw2::SHAPE_SIZE::UNKNOWN) 
      {
        break; 
      }
    }

  }
  else
  {
    RCLCPP_WARN(L, "Error moving to mystery shape");
    return;
  }


  if (reference_shape_1.type != cw2::SHAPE_TYPE::UNKNOWN && reference_shape_1.type == mystery_shape.type)
  {
    response->mystery_object_num = 1;
    RCLCPP_INFO(node_->get_logger(), "Mystery shape matches Reference 1!");
  }
  else if (reference_shape_2.type != cw2::SHAPE_TYPE::UNKNOWN && reference_shape_2.type == mystery_shape.type)
  {
    response->mystery_object_num = 2;
    RCLCPP_INFO(node_->get_logger(), "Mystery shape matches Reference 2!");

  }
  else
  {
    /// something didn't work :(
    if (reference_shape_1.type == cw2::SHAPE_TYPE::UNKNOWN)
    {
      RCLCPP_INFO(node_->get_logger(), "Reference 1 is Unknown");
    }

    if (reference_shape_2.type == cw2::SHAPE_TYPE::UNKNOWN)
    {
      RCLCPP_INFO(node_->get_logger(), "Reference 2 is Unknown");
    }

    if (mystery_shape.type == cw2::SHAPE_TYPE::UNKNOWN)
    {
      RCLCPP_INFO(node_->get_logger(), "Mystery shape is Unknown");
    }
  }
  
  
}


void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  (void)request;
  auto L = node_->get_logger();
  RCLCPP_INFO(L, "Starting Task 3: Explicit Perimeter Sweep...");

  arm_group_->setMaxVelocityScalingFactor(0.5);
  arm_group_->setMaxAccelerationScalingFactor(0.5);

  // static const std::string planning_group = "panda_arm";
  // moveit::planning_interface::MoveGroupInterface move_group3(node_, planning_group);

  // move_group3.setPlanningTime(10.0); //maybe chabge that to 5 seconds
  // move_group3.setMaxVelocityScalingFactor(0.2);
  // move_group3.setMaxAccelerationScalingFactor(0.2);
  
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


  std::array<std::array<double, 3>, 16> target_poses = {{
    {-0.45, 0.00, 0.65},
    {-0.45, 0.17, 0.65},
    {-0.45, 0.35, 0.65},
    {-0.23, 0.35, 0.65},
    {0.00, 0.35, 0.65},
    {0.23, 0.35, 0.65},
    {0.45, 0.35, 0.65},
    {0.45, 0.17, 0.65},
    {0.45, 0.00, 0.65},
    {0.45, -0.17, 0.65},
    {0.45, -0.35, 0.65},
    {0.23, -0.35, 0.65},
    {0.00, -0.35, 0.65},
    {-0.23, -0.35, 0.65},
    {-0.45, -0.35, 0.65},
    {-0.45, -0.17, 0.65},


  }};
  
  std::vector<PointCPtr> clusters;

  std::vector<cw2::SHAPE> noughts;
  std::vector<cw2::SHAPE> crosses;
  std::vector<Eigen::Vector3f> obstacle_locations;
  Eigen::Vector3f basket_location;

  std::vector<Eigen::Vector3f> all_locations;

  for (const auto& pose : target_poses)
  {
    target_pose.position.x = pose[0];
    target_pose.position.y = pose[1];
    target_pose.position.z = pose[2];



    // move_group3.setPoseTarget(target_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // if(move_group3.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    // {
    //   move_group3.execute(plan);
    if (cart_move(arm_group_, target_pose, L, "Scanning Move"))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      clusters = findClusters();

      for (size_t i = 0; i < clusters.size(); i++)
      {
        RCLCPP_INFO(node_->get_logger(), "Classifyting Cluster %ld", i);

        

        std::string color = colorOfPointCloud(*clusters[i], 0.3);

        cw2::SHAPE shape = classifyShape(clusters[i]);

        if (color == "black")
        {
          obstacle_locations.push_back(toWorldFrame(getCentroid(clusters[i])));
          all_locations.push_back(toWorldFrame(getCentroid(clusters[i])));
          RCLCPP_INFO(node_->get_logger(), "Classified as obstacle");
        }
        else if (color == "brown" && shape.type == cw2::SHAPE_TYPE::NOUGHT && shape.size == cw2::SHAPE_SIZE::UNKNOWN)
        {
          basket_location = toWorldFrame(getCentroid(clusters[i]));
          all_locations.push_back(basket_location);
          RCLCPP_INFO(node_->get_logger(), "Classified as a basket");
        }
        else if (shape.type != cw2::SHAPE_TYPE::UNKNOWN && shape.size != cw2::SHAPE_SIZE::UNKNOWN)
        { 

          bool is_duplicate = false;

          // check to see if cluster is within 20mm of something else saved
          for (const auto& pt : all_locations) {
            if ((shape.centroid - pt).norm() < 0.100) {
                is_duplicate = true;
                RCLCPP_INFO (L, "oupsie daisy!! we have a duplicate centroid. I won't save that!");
                break;
            }
          }

          if (!is_duplicate)
          {
            if (shape.type == cw2::SHAPE_TYPE::NOUGHT)
            {
              noughts.push_back(shape);
            }
            else if (shape.type == cw2::SHAPE_TYPE::CROSS)
            {
              crosses.push_back(shape);
            }

            all_locations.push_back(shape.centroid);
          }
  
        }

      }

    }
    else
    {
      RCLCPP_WARN(L, "Error During Motion Planning of pose {%.2f, %.2f, %.2f}", pose[0], pose[1], pose[2]);
    }

    
    RCLCPP_INFO (L, " %ld crosses, %ld noughts, fiiiiive golden riiings", crosses.size(), noughts.size()); 

  }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    int num_crosses = (int) crosses.size();
    int num_noughts = (int) noughts.size();
    
    RCLCPP_INFO (L, " %d crosses, %d noughts, fiiiiive golden riiings", num_crosses, num_noughts); 
    std::sort(crosses.begin(), crosses.end(), [](cw2::SHAPE x, cw2::SHAPE y) 
		{
			return x.size > y.size;
    });

    std::sort(noughts.begin(), noughts.end(), [](cw2::SHAPE x, cw2::SHAPE y) 
		{
			return x.size > y.size;
    });

    response->total_num_shapes = num_crosses + num_noughts;

    cw2::SHAPE shape_to_pick;

    if (num_crosses >= num_noughts)
    {
      response->num_most_common_shape = num_crosses;
      shape_to_pick = crosses.front();
    }
    else
    {
      response->num_most_common_shape = num_noughts;
      shape_to_pick = noughts.front();
    }

    for (const auto& pt : noughts) {
      RCLCPP_INFO(L, "Nought at {%.2f, %.2f, %.2f}", pt.centroid.x(), pt.centroid.y(), pt.centroid.z());
    }

    for (const auto& pt : crosses) {
      RCLCPP_INFO(L, "Cross at {%.2f, %.2f, %.2f}", pt.centroid.x(), pt.centroid.y(), pt.centroid.z());
    }



  double grip_z = shape_to_pick.centroid.z() - 0.040 + 0.125;
  double basket_z = basket_location.z() -0.050 + 0.35;
  double safe_z = shape_to_pick.centroid.z() + 0.65;
  double ox = shape_to_pick.centroid.x();
  double oy = shape_to_pick.centroid.y();
  double gx = basket_location.x();
  double gy  = basket_location.y();




  moveit::planning_interface::MoveGroupInterface hand_group(node_, "hand");
  hand_group.setMaxVelocityScalingFactor(1.0);
  hand_group.setMaxAccelerationScalingFactor(1.0);

  auto fail = [&]() {
    open_gripper(hand_group_, L);
    go_home(arm_group_, L);
  };

  
  go_home(arm_group_, L);
  // Move 1
  RCLCPP_INFO(L, "Move A: Moving high above shape");
  if (!joint_move(arm_group_, td_pose(ox, oy, safe_z), L, "Move A")) { fail(); return; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  if (!open_gripper(hand_group_, L)) { fail(); return; }

  // Determine shape yaw
  

  double shape_yaw_rad = shape_to_pick.yaw * (M_PI / 180.0);
  double gripper_yaw = shape_yaw_rad - M_PI/4.0;

  double gx_pick = ox;
  double gy_pick = oy;
  double gx_drop = gx;
  double gy_drop = gy;

  if (shape_to_pick.type == cw2::SHAPE_TYPE::NOUGHT) {
    if (shape_to_pick.size == cw2::SHAPE_SIZE::MM_40){
    RCLCPP_INFO(L, "Applying nought offsets");
    gx_pick = ox + 0.08 * std::sin(shape_yaw_rad);
    gy_pick = oy - 0.08 * std::cos(shape_yaw_rad);
    gx_drop = gx;
    gy_drop = gy - 0.05;
    } else if ( shape_to_pick.size == cw2::SHAPE_SIZE::MM_30){
    RCLCPP_INFO(L, "Applying nought offsets");
    gx_pick = ox + 0.06 * std::sin(shape_yaw_rad);
    gy_pick = oy - 0.06 * std::cos(shape_yaw_rad);
    gx_drop = gx;
    gy_drop = gy - 0.05;
    } else if (shape_to_pick.size == cw2::SHAPE_SIZE::MM_20){
      RCLCPP_INFO(L, "Applying nought offsets");
    gx_pick = ox + 0.05 * std::sin(shape_yaw_rad);
    gy_pick = oy - 0.05 * std::cos(shape_yaw_rad);
    gx_drop = gx;
    gy_drop = gy - 0.05;
    }
  } else {
    if (shape_to_pick.size == cw2::SHAPE_SIZE::MM_40){
    RCLCPP_INFO(L, "Applying cross offsets");
    gx_pick = ox + 0.05 * std::cos(shape_yaw_rad);
    gy_pick = oy + 0.05 * std::sin(shape_yaw_rad);
    gx_drop = gx + 0.02;
    gy_drop = gy;
    } else if (shape_to_pick.size == cw2::SHAPE_SIZE::MM_30){
    RCLCPP_INFO(L, "Applying cross offsets");
    gx_pick = ox + 0.04 * std::cos(shape_yaw_rad);
    gy_pick = oy + 0.04 * std::sin(shape_yaw_rad);
    gx_drop = gx + 0.02;
    gy_drop = gy;
    } else {
      RCLCPP_INFO(L, "Applying cross offsets");
    gx_pick = ox + 0.0125 * std::cos(shape_yaw_rad);
    gy_pick = oy + 0.0125 * std::sin(shape_yaw_rad);
    gx_drop = gx + 0.02;
    gy_drop = gy;
    }

    
  }
  

  // Move 2
  RCLCPP_INFO(L, "Move B: Descending to grip");
  arm_group_->setMaxVelocityScalingFactor(0.1);
  arm_group_->setMaxAccelerationScalingFactor(0.1);
  if (!cart_move(arm_group_, td_pose(gx_pick, gy_pick, grip_z, gripper_yaw), L, "Move B")) { fail(); return; }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  //Move 3

  //OLD GRIP//
  // RCLCPP_INFO(L, "Move C: Closing gripper. I WANT THE BIG BUNNY");
  // strong_grip(hand_group_, L, 40);  // Task 1 always x=40mm

   //I think this grip works faster
  RCLCPP_INFO(L, "Closing gripper. TOUT TOUT TOUT. I WANT THE BIG BUNNY");
  hand_group.setJointValueTarget("panda_finger_joint1", 0.020); 
  moveit::planning_interface::MoveGroupInterface::Plan close_plan;
  bool success = (hand_group.plan(close_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  hand_group.execute(close_plan);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  //Move 4
  RCLCPP_INFO(L, "Move D: Lifting object to safe height");
  arm_group_->setMaxVelocityScalingFactor(0.5);
  arm_group_->setMaxAccelerationScalingFactor(0.5);
  if (!cart_move(arm_group_, td_pose(gx_pick, gy_pick, safe_z, gripper_yaw), L, "Move D")) { fail(); return; }
  //std::this_thread::sleep_for(std::chrono::milliseconds(200));

  //Move 5
  RCLCPP_INFO(L, "Move E: Translating over obstacles to basket");
  if (!joint_move(arm_group_, td_pose(gx_drop, gy_drop, safe_z), L, "Move E")) { fail(); return; }
  //std::this_thread::sleep_for(std::chrono::milliseconds(200));

  //Move 6
  RCLCPP_INFO(L, "Move F: Descending into basket");
   if (!cart_move(arm_group_, td_pose(gx_drop, gy_drop, basket_z), L, "Move F")) { fail(); return; }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  

  // //Move 7

  // if (!open_gripper(hand_group_, L)) { fail(); return; }

  //OLD RELEASE//
  // RCLCPP_INFO(L, "Move G: KOBE! Releasing shape");
  // moveit::planning_interface::MoveGroupInterface hand_group(node_, "hand");
  // hand_group.setMaxVelocityScalingFactor(1.0);
  // hand_group.setMaxAccelerationScalingFactor(1.0);
  // moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
  // hand_group.setNamedTarget("open");
  // bool success = (hand_group.plan(hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  // hand_group.execute(hand_plan);
  // std::this_thread::sleep_for(std::chrono::milliseconds(200));


  //I think this release works faster
  RCLCPP_INFO(L, "KOBE!");
  hand_group.setNamedTarget("open");
  moveit::planning_interface::MoveGroupInterface::Plan final_open_plan;
  success = (hand_group.plan(final_open_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  hand_group.execute(final_open_plan);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // RCLCPP_INFO(L, "Move E: Translating over obstacles to basket");
  // if (!joint_move(arm_group_, td_pose(gx_drop, gy_drop, safe_z), L, "Move E")) { fail(); return; }
  // std::this_thread::sleep_for(std::chrono::milliseconds(200));


  //Move 8
  RCLCPP_INFO(L, "Move F: Ascending above basket (Oogway ascends!)");
  if (!cart_move(arm_group_, td_pose(gx_drop, gy_drop, safe_z), L, "Move F")) { fail(); return; }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  //Move 9
  RCLCPP_INFO(L, "Returning to 'ready' home position...");
  go_home(arm_group_, L);
  std::this_thread::sleep_for(std::chrono::milliseconds(500)); 



  RCLCPP_INFO(L, "=== Task 3 complete ===");
  

}

//PCL FUNCTIONS

void cw2::rosTopicToCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_input_msg)
{

  std::lock_guard<std::mutex> lock(cloud_mutex_);

  g_input_pc_frame_id = cloud_input_msg->header.frame_id;

  pcl::fromROSMsg(*cloud_input_msg, *g_cloud_ptr);

  *g_cloud_filtered = *g_cloud_ptr;

  RCLCPP_INFO_STREAM(node_->get_logger(), "saved cloud");
}

void cw2::applyVoxelGrid(double g_leaf_size)
{
  
  PointCPtr output_cloud(new PointC);

  g_vx.setInputCloud(g_cloud_filtered);
  g_vx.setLeafSize(g_leaf_size, g_leaf_size, g_leaf_size);
  g_vx.filter(*output_cloud);

  g_cloud_filtered = (output_cloud);

}

void cw2::applyPassthrough(double g_pass_min, double g_pass_max, std::string g_pass_axis, PointCPtr &in_cloud_ptr)
{
  
  PointCPtr output_cloud(new PointC);

  g_pt.setInputCloud(in_cloud_ptr);
  g_pt.setFilterFieldName(g_pass_axis);
  g_pt.setFilterLimits(g_pass_min, g_pass_max);
  g_pt.filter(*output_cloud);
  in_cloud_ptr.swap(output_cloud);


}

void cw2::applyOutlierRemoval(int g_outlier_mean_k, double g_outlier_stddev)
{
  
  PointCPtr output_cloud(new PointC);

  g_sor.setInputCloud(g_cloud_filtered);
  g_sor.setMeanK(g_outlier_mean_k);
  g_sor.setStddevMulThresh(g_outlier_stddev);
  g_sor.filter(*output_cloud);
  g_cloud_filtered = (output_cloud);

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
  // g_extract_normals.setNegative(true);
  // g_extract_normals.setInputCloud(g_cloud_normals);
  // g_extract_normals.setIndices(g_inliers_plane);
  // g_extract_normals.filter(*g_cloud_segmented_normals);
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


Eigen::Vector3f cw2::getCentroid(PointCPtr &in_cloud_ptr)
{
  
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid);

  return centroid.head<3>();  // drops the homogeneous w component
}

cw2::SHAPE cw2::classifyShape(PointCPtr &in_cloud_ptr)
{

  cw2::SHAPE shape = {cw2::SHAPE_TYPE::UNKNOWN, cw2::SHAPE_SIZE::UNKNOWN, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0};

  g_inertia_estimator.setInputCloud(in_cloud_ptr);
  g_inertia_estimator.compute();

  PointT min_OBB, max_OBB, position_OBB;
  Eigen::Matrix3f rotation_OBB;
  float major_eigen, middle_eigen, minor_eigen;

  g_inertia_estimator.getOBB(min_OBB, max_OBB, position_OBB, rotation_OBB);
  g_inertia_estimator.getEigenValues(major_eigen, middle_eigen, minor_eigen);

  // Print eigenvalues (major, middle, minor)
  RCLCPP_INFO(node_->get_logger(), "Eigenvalues: major=%.5f middle=%.5f minor=%.5f", major_eigen, middle_eigen, minor_eigen);

  RCLCPP_INFO(node_->get_logger(), "Delta: x=%.1f y=%.1f z=%.1f", 1000*abs(max_OBB.x - min_OBB.x), 1000*abs(max_OBB.y - min_OBB.y), 1000*abs(max_OBB.z - min_OBB.z));

  //sanity checks
  //if either the major or middle eigen is zero, skip this cluster
  if (major_eigen < 0.0001 || middle_eigen < 0.0001)
  {
    RCLCPP_INFO(node_->get_logger(), "Zero Major/Middle eigen, exiting");
    return shape;
  }

  //if major != middle eigen. Have a tolerance of 10%. 
  //removes clusters that are not squares
  if (abs((major_eigen/middle_eigen) - 1) > 0.1)
  {
    RCLCPP_INFO(node_->get_logger(), "Nonsquare eigen, exiting");
    return shape;
  }

  int num_points_close = 0;
  const float close_point_threshold = 0.020f;

  Eigen::Vector3f centroid_vec(position_OBB.x, position_OBB.y, position_OBB.z);
  double sum_dist = 0.0;
  double closest_distance = std::numeric_limits<float>::max();
  Eigen::Vector3f closest_point;
  for (const auto& pt : in_cloud_ptr->points) {
    Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
    double dist = (pt_vec - centroid_vec).norm();
    sum_dist += dist;
    if (dist < close_point_threshold)
    {
      num_points_close++;
    }
    if (dist < closest_distance)
    {
      closest_distance = dist;
      closest_point = (pt_vec - centroid_vec);
    }

  }
  double avg_dist = (in_cloud_ptr->size() > 0) ? (sum_dist / in_cloud_ptr->size()) : 0.0;


  //SHAPE CLASSIFICATION

  if (num_points_close > 500)
  {
    shape.type = cw2::SHAPE_TYPE::CROSS;
    RCLCPP_INFO(node_->get_logger(), "%d central points, assigned as Cross", num_points_close);

  }
  else
  {
    shape.type = cw2::SHAPE_TYPE::NOUGHT;
    RCLCPP_INFO(node_->get_logger(), "%d central points, assigned as Nought", num_points_close);
  }


  //SIZE CLASSIFICATION

  float delta = 1000*(abs(max_OBB.x - min_OBB.x) + abs(max_OBB.y - min_OBB.y))/2; //convert from m to mm

  if (shape.type == cw2::SHAPE_TYPE::NOUGHT)
  {
    if (avg_dist >= 0.035 && avg_dist < 0.055)
    {
      shape.size = cw2::SHAPE_SIZE::MM_20;
      RCLCPP_INFO(node_->get_logger(), "Average Distance %.5f, assigned as 20mm", avg_dist);
    }
    else if (avg_dist >= 0.055 && avg_dist < 0.075)
    {
      shape.size = cw2::SHAPE_SIZE::MM_30;
      RCLCPP_INFO(node_->get_logger(), "Average Distance %.5f, assigned as 30mm", avg_dist);
    }
    else if (avg_dist >= 0.075 && avg_dist < 0.095)
    {
      shape.size = cw2::SHAPE_SIZE::MM_40;
      RCLCPP_INFO(node_->get_logger(), "Average Distance %.5f, assigned as 40mm", avg_dist);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Average distance %.5f too big, exiting", avg_dist);
      return shape;
    }

  }
  else
  {
    
    if (delta > 75 && delta < 125)
    {
      shape.size = cw2::SHAPE_SIZE::MM_20;
      
      RCLCPP_INFO(node_->get_logger(), "Delta x %.3f, assigned as 20mm", delta);
    }
    else if (delta >= 125 && delta < 175)
    {
      shape.size = cw2::SHAPE_SIZE::MM_30;
      RCLCPP_INFO(node_->get_logger(), "Delta x %.3f, assigned as 30mm", delta);
    }
    else if (delta >= 175 && delta < 300)
    {
      shape.size = cw2::SHAPE_SIZE::MM_40;
      RCLCPP_INFO(node_->get_logger(), "Delta x %.3f, assigned as 40mm", delta);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Delta x %.3f too big, exiting", delta);
      return shape;
    }
  }

  // CENTROID CLASSIFICATION, world frame

  shape.centroid = toWorldFrame(getCentroid(in_cloud_ptr));
  
  //YAW CLASSIFICATION

  double yaw;
  if (shape.type == cw2::SHAPE_TYPE::NOUGHT)
  { 
    yaw = atan2(closest_point.y(), closest_point.x());
  }
  else
  {
    yaw = atan2(rotation_OBB(1, 0), rotation_OBB(0, 0));
  }

  // Print yaw
  RCLCPP_INFO(node_->get_logger(), "Yaw: %.3f", 180*yaw/M_PI);

  double adjusted_yaw = (int)(90 - (180*yaw/M_PI))%90;

  if (adjusted_yaw < 0)
  {
    adjusted_yaw = adjusted_yaw + 90;
  }

  RCLCPP_INFO(node_->get_logger(), "adjusted yaw: %.3f", adjusted_yaw);

  shape.yaw = adjusted_yaw;

  // DEBUG ONLY BELOW
  tf2::Quaternion q;
  q.setRPY(0, 0, adjusted_yaw);

  geometry_msgs::msg::Point p;
  p.x = position_OBB.x;
  p.y = position_OBB.y;
  p.z = position_OBB.z;

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation = tf2::toMsg(q);
  pose.pose.position = p;

  pose.header.frame_id = g_input_pc_frame_id;
  pose.header.stamp = latest_cloud_msg_->header.stamp;

  if (g_pub_pose)
  {
    g_pub_pose->publish(pose);
  } 

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
  applyPassthrough(pcl_pass_min_, pcl_pass_max_, pcl_pass_axis_, g_cloud_filtered);
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
  rosTopicToCloud(latest_cloud_msg_);
  // applyVoxelGrid(0.05);
  //applyPassthrough(-0.31, 0.18, "y");
  applyOutlierRemoval(20, 0.75);
  findNormals(25);
  segmentationPipeline(0.1, 100, 0.01);
}

std::vector<PointCPtr> cw2::findClusters()
{

  //apply outlier detection, remove plane
  filteringPipeline();
  
  double plane_z = getCentroid(g_cloud_plane).z();
  //remove everything below the segmented plane

  applyPassthrough(0, plane_z, "z", g_cloud_segmented_plane);

  RCLCPP_INFO(node_->get_logger(), "Points in Clusters %ld", (*g_cloud_segmented_plane).size());

  return extractEuclideanClusters(pcl_cluster_tolerance_, pcl_cluster_min_size_, pcl_cluster_max_size_);

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
