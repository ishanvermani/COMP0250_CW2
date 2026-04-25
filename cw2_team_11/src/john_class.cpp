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

static geometry_msgs::msg::Pose td_pose(double x, double y, double z, double target_yaw = 0.0)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y; p.position.z = z;
  double roll = M_PI, pitch = 0.0;
  double yaw = target_yaw - M_PI/4.0;
  double hr = roll/2, hp = pitch/2, hy = yaw/2;
  p.orientation.w = cos(hr)*cos(hp)*cos(hy) + sin(hr)*sin(hp)*sin(hy);
  p.orientation.x = sin(hr)*cos(hp)*cos(hy) - cos(hr)*sin(hp)*sin(hy);
  p.orientation.y = cos(hr)*sin(hp)*cos(hy) + sin(hr)*cos(hp)*sin(hy);
  p.orientation.z = cos(hr)*cos(hp)*sin(hy) - sin(hr)*sin(hp)*cos(hy);
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

static void strong_grip(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &h,
  const rclcpp::Logger &l, int cell_size_mm = 40)
{
  double arm_half_m = (cell_size_mm / 2.0) / 1000.0;
  double target = std::max(0.001, arm_half_m - 0.010);
  h->setMaxVelocityScalingFactor(0.1);
  h->setMaxAccelerationScalingFactor(0.1);
  h->setJointValueTarget("panda_finger_joint1", target);
  h->setJointValueTarget("panda_finger_joint2", target);
  moveit::planning_interface::MoveGroupInterface::Plan p;
  if (h->plan(p) == moveit::core::MoveItErrorCode::SUCCESS) h->execute(p);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
  tf_listener_(tf_buffer_),
  g_cloud_ptr(new PointC)
{
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
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*msg, pcl_cloud);

  PointCPtr latest_cloud(new PointC);
  pcl::fromPCLPointCloud2(pcl_cloud, *latest_cloud);

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  g_input_pc_frame_id_ = msg->header.frame_id;
  g_cloud_ptr = std::move(latest_cloud);
  ++g_cloud_sequence_;
}

// ── Shape property detection (EXACT copy from working code) ──────────
static void get_shape_properties(const PointCPtr& cloud, double ox, double oy, double oz, const std::string& shape_type, const rclcpp::Logger &l, double &out_yaw, double &out_size) {
  out_yaw = 0.0; out_size = 0.04; 
  if (!cloud || cloud->empty()) return;
  pcl::CropBox<PointT> crop; crop.setInputCloud(cloud);
  // Tight crop: max shape is 5*40=200mm, so ±120mm captures any shape with margin
  // but excludes distant floor/noise that inflates the bounding box
  crop.setMin(Eigen::Vector4f(ox - 0.12, oy - 0.12, 0.025, 1.0));
  crop.setMax(Eigen::Vector4f(ox + 0.12, oy + 0.12, 0.06, 1.0));
  PointCPtr cropped(new PointC); crop.filter(*cropped);
  if (cropped->empty()) return;

  double best_yaw = 0.0;
  double best_area = (shape_type == "cross") ? -1.0 : 1e9;

  for (int a = 0; a < 90; ++a) {
    double rad = a * M_PI / 180.0;
    double min_x = 1e6, max_x = -1e6, min_y = 1e6, max_y = -1e6;
    for (const auto& pt : cropped->points) {
      double dx = pt.x - ox, dy = pt.y - oy;
      double rx = dx * cos(-rad) - dy * sin(-rad);
      double ry = dx * sin(-rad) + dy * cos(-rad);
      min_x = std::min(min_x, rx); max_x = std::max(max_x, rx);
      min_y = std::min(min_y, ry); max_y = std::max(max_y, ry);
    }
    double area = (max_x - min_x) * (max_y - min_y);
    bool is_better = (shape_type == "cross") ? (area > best_area) : (area < best_area);
    if (is_better) {
      best_area = area; best_yaw = rad;
      out_size = std::max(max_x - min_x, max_y - min_y) / 5.0;
    }
  }
  out_yaw = best_yaw;
  RCLCPP_INFO(l, "Shape: yaw=%.1f deg, cell=%.1f mm", out_yaw * 180.0 / M_PI, out_size * 1000);
}

///////////////////////////////////////////////////////////////////////////////
// Reusable pick-and-place (EXACT working logic, usable by Task 1 and Task 3)
///////////////////////////////////////////////////////////////////////////////

bool cw2::pick_and_place_shape(
  double ox, double oy, double oz,
  double bx, double by, double bz,
  const std::string &shape_type,
  int cell_size_mm, double shape_yaw)
{
  auto L = node_->get_logger();
  double shape_size = cell_size_mm / 1000.0;

  // EXACT offset logic from working code
  double px, py;
  if (shape_type == "nought") {
    double mult = (cell_size_mm <= 20) ? 2.5 : 2.0;
    double offset = mult * shape_size;
    px = ox + (offset * std::sin(shape_yaw));
    py = oy - (offset * std::cos(shape_yaw));
  } else {
    px = ox + 1.25 * shape_size;  // fixed +X offset, no rotation
    py = oy;                       // no Y offset
  }

  // EXACT height logic — hardcoded, oz from TF is unreliable
  double grip_z = ft2l8(0.06), safe_z = 0.55, basket_z = bz + 0.15 + FTIP;

  RCLCPP_INFO(L, "pick: shape=%s size=%d yaw=%.1f grasp=(%.3f,%.3f) grip_z=%.3f",
              shape_type.c_str(), cell_size_mm, shape_yaw*180/M_PI, px, py, grip_z);

  auto fail = [&]() { open_gripper(hand_group_, L); go_home(arm_group_, L); };

  double yaw_for_grip = (shape_type == "nought") ? shape_yaw : 0.0;

  go_home(arm_group_, L);
  if (!joint_move(arm_group_, td_pose(px, py, safe_z, yaw_for_grip), L, "Above")) { fail(); return false; }
  open_gripper(hand_group_, L);
  arm_group_->setMaxVelocityScalingFactor(0.1);
  if (!cart_move(arm_group_, td_pose(px, py, grip_z, yaw_for_grip), L, "Descend")) { fail(); return false; }
  strong_grip(hand_group_, L, cell_size_mm);

  arm_group_->setMaxVelocityScalingFactor(0.5);
  if (!cart_move(arm_group_, td_pose(px, py, safe_z, yaw_for_grip), L, "Lift")) { fail(); return false; }
  if (!joint_move(arm_group_, td_pose(bx, by, safe_z, 0.0), L, "Over Basket")) { fail(); return false; }
  if (!cart_move(arm_group_, td_pose(bx, by, basket_z, 0.0), L, "Lower")) { fail(); return false; }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  open_gripper(hand_group_, L);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // Small lift to clear the shape before full retreat
  cart_move(arm_group_, td_pose(bx, by, basket_z + 0.05, 0.0), L, "Clear shape");
  cart_move(arm_group_, td_pose(bx, by, safe_z, 0.0), L, "Retreat");
  go_home(arm_group_, L);
  RCLCPP_INFO(L, "pick_and_place: DONE");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Task 1 (EXACT working logic from the code that picks up x=20)
///////////////////////////////////////////////////////////////////////////////

void cw2::t1_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> response)
{
  (void)response; auto L = node_->get_logger();
  RCLCPP_INFO(L, "=== Task 1 ===");

  geometry_msgs::msg::PointStamped obj_world, goal_world;
  try {
    obj_world = tf_buffer_.transform(request->object_point, "panda_link0", tf2::durationFromSec(2.0));
    goal_world = tf_buffer_.transform(request->goal_point, "panda_link0", tf2::durationFromSec(2.0));
  } catch (...) { RCLCPP_ERROR(L, "TF failed"); return; }

  const double ox = obj_world.point.x, oy = obj_world.point.y, oz = obj_world.point.z;
  const double gx = goal_world.point.x, gy = goal_world.point.y, gz = goal_world.point.z;
  const std::string shape = request->shape_type;

  RCLCPP_INFO(L, "Shape: %s at (%.3f, %.3f, %.3f)", shape.c_str(), ox, oy, oz);
  RCLCPP_INFO(L, "Basket at (%.3f, %.3f, %.3f)", gx, gy, gz);

  // Detect shape yaw and size from point cloud (EXACT working logic)
  double shape_yaw = 0.0, shape_size = 0.04;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (g_cloud_ptr && !g_cloud_ptr->empty()) {
      try {
        auto tf_cam = tf_buffer_.lookupTransform("panda_link0", g_input_pc_frame_id_, tf2::TimePointZero);
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        trans.translation() << tf_cam.transform.translation.x, tf_cam.transform.translation.y, tf_cam.transform.translation.z;
        trans.rotate(Eigen::Quaternionf(tf_cam.transform.rotation.w, tf_cam.transform.rotation.x, tf_cam.transform.rotation.y, tf_cam.transform.rotation.z));
        PointCPtr world_cloud(new PointC);
        pcl::transformPointCloud(*g_cloud_ptr, *world_cloud, trans);
        get_shape_properties(world_cloud, ox, oy, oz, shape, L, shape_yaw, shape_size);
      } catch (...) { RCLCPP_WARN(L, "Detection failed, using defaults"); }
    }
  }

  // shape_size from detector = bounding_box_width / 5 = cell estimate
  // But detector returns inflated values (60mm for x=20 shape!)
  // Fix: use total width (shape_size * 5) with wider thresholds
  double total_width = shape_size * 5.0;
  RCLCPP_INFO(L, "Detector raw: cell=%.1fmm, total=%.0fmm", shape_size*1000, total_width*1000);

  // Snap based on total width:
  //   x=20 → total ~100mm → anything < 125mm
  //   x=30 → total ~150mm → 125-175mm
  //   x=40 → total ~200mm → 175mm+
  int cell_size_mm;
  if (total_width < 0.125)      cell_size_mm = 20;
  else if (total_width < 0.175) cell_size_mm = 30;
  else                          cell_size_mm = 40;

  RCLCPP_INFO(L, "cell=%dmm, yaw=%.1fdeg", cell_size_mm, shape_yaw * 180.0 / M_PI);

  bool ok = pick_and_place_shape(ox, oy, oz, gx, gy, gz, shape, cell_size_mm, shape_yaw);
  if (ok) RCLCPP_INFO(L, "=== Task 1 complete ===");
  else RCLCPP_ERROR(L, "=== Task 1 FAILED ===");
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
