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


static geometry_msgs::msg::Pose td_pose(double x, double y, double z, double yaw = 0.0)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y; p.position.z = z;

  
  tf2::Quaternion q_down;
  q_down.setRPY(M_PI, 0.0, 0.0); 

  
  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, yaw);

  
  tf2::Quaternion q_final = q_yaw * q_down;
  q_final.normalize();

  p.orientation.x = q_final.x();
  p.orientation.y = q_final.y();
  p.orientation.z = q_final.z();
  p.orientation.w = q_final.w();
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
  const rclcpp::Logger &l)
{
  
  h->setMaxVelocityScalingFactor(0.05);
  h->setMaxAccelerationScalingFactor(0.05);
  RCLCPP_INFO(l, "  GRIP (j1=0.012)");
  
  
  h->setJointValueTarget("panda_finger_joint1", 0.012);
  h->setJointValueTarget("panda_finger_joint2", 0.012);
  
  moveit::planning_interface::MoveGroupInterface::Plan p;
  if (h->plan(p) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(l, "  Grip plan failed"); return;
  }
  h->execute(p);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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


static double get_shape_yaw(const PointCPtr& cloud, double ox, double oy, double oz, const rclcpp::Logger &l) {
  if (!cloud || cloud->empty()) {
    RCLCPP_WARN(l, "Point cloud is empty! Defaulting yaw to 0.");
    return 0.0;
  }

  
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(ox - 0.25, oy - 0.25, oz - 0.015, 1.0));
  crop.setMax(Eigen::Vector4f(ox + 0.25, oy + 0.25, oz + 0.025, 1.0));

  PointCPtr cropped_cloud(new PointC);
  crop.filter(*cropped_cloud);

  if (cropped_cloud->empty()) {
    RCLCPP_WARN(l, "No points found in crop box! Defaulting yaw to 0.");
    return 0.0;
  }

  
  double best_yaw = 0.0;
  double min_area = std::numeric_limits<double>::max();

  
  for (int angle_deg = 0; angle_deg < 90; ++angle_deg) {
    double angle_rad = angle_deg * M_PI / 180.0;
    double min_x = 1e6, max_x = -1e6;
    double min_y = 1e6, max_y = -1e6;

    for (const auto& pt : cropped_cloud->points) {
      // Translate point to origin
      double dx = pt.x - ox;
      double dy = pt.y - oy;
      
      // Rotate point to test alignment
      double rx = dx * cos(-angle_rad) - dy * sin(-angle_rad);
      double ry = dx * sin(-angle_rad) + dy * cos(-angle_rad);

      if (rx < min_x) min_x = rx;
      if (rx > max_x) max_x = rx;
      if (ry < min_y) min_y = ry;
      if (ry > max_y) max_y = ry;
    }

    double area = (max_x - min_x) * (max_y - min_y);
    if (area < min_area) {
      min_area = area;
      best_yaw = angle_rad;
    }
  }

  RCLCPP_INFO(l, "Calculated Shape Yaw: %.2f degrees", best_yaw * 180.0 / M_PI);
  return best_yaw;
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

  // rotation and offset logicx
  
  double shape_yaw = 0.0; 
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (!g_cloud_ptr->empty() && !g_input_pc_frame_id_.empty()) {
      try {
        // Find how the camera is positioned relative to the robot base
        geometry_msgs::msg::TransformStamped tf_cam_to_base = 
          tf_buffer_.lookupTransform("panda_link0", g_input_pc_frame_id_, tf2::TimePointZero);

        // Convert the ROS transform to a mathematical matrix (Eigen)
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << tf_cam_to_base.transform.translation.x,
                                   tf_cam_to_base.transform.translation.y,
                                   tf_cam_to_base.transform.translation.z;
        Eigen::Quaternionf q(
          tf_cam_to_base.transform.rotation.w,
          tf_cam_to_base.transform.rotation.x,
          tf_cam_to_base.transform.rotation.y,
          tf_cam_to_base.transform.rotation.z);
        transform.rotate(q);

        
        PointCPtr transformed_cloud(new PointC);
        pcl::transformPointCloud(*g_cloud_ptr, *transformed_cloud, transform);

        
        shape_yaw = get_shape_yaw(transformed_cloud, ox, oy, oz, L);

      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(L, "TF Exception: Could not align point cloud! Defaulting to 0.0. Error: %s", ex.what());
      }
    } else {
      RCLCPP_WARN(L, "Point cloud or frame ID is empty! Defaulting yaw to 0.0.");
    }
  }

  const double arm_offset = 0.08;  // 80mm from centroid
  double gx_pick = ox;
  double gy_pick = oy;

  
  if (shape == "nought") {
    gx_pick -= arm_offset * std::sin(shape_yaw);
    gy_pick += arm_offset * std::cos(shape_yaw);
  } else if (shape == "cross") {
    gx_pick += arm_offset * std::cos(shape_yaw);
    gy_pick += arm_offset * std::sin(shape_yaw);
  }

  RCLCPP_INFO(L, "Grasp point dynamic offset applied: (%.4f, %.4f) [yaw=%.2f]",
              gx_pick, gy_pick, shape_yaw);

  // Height calculations
  const double shape_centroid_z = 0.020;           
  const double grasp_l8     = ft2l8(shape_centroid_z + 0.015); 
  const double pre_grasp_l8 = grasp_l8 + 0.085;    
  const double transit_l8   = 0.40;                 
  const double release_l8   = ft2l8(gz + 0.10);    

  RCLCPP_INFO(L, "Heights: grasp_l8=%.4f pre_grasp=%.4f transit=%.4f release=%.4f",
              grasp_l8, pre_grasp_l8, transit_l8, release_l8);

  auto fail = [&]() {
    open_gripper(hand_group_, L);
    go_home(arm_group_, L);
  };

  // ─picking up shape

  go_home(arm_group_, L);

  
  if (!joint_move(arm_group_, td_pose(gx_pick, gy_pick, transit_l8, shape_yaw), L, "Above shape")) {
    fail(); return;
  }

  
  if (!open_gripper(hand_group_, L)) { fail(); return; }

  
  if (!cart_move(arm_group_, td_pose(gx_pick, gy_pick, pre_grasp_l8, shape_yaw), L, "Pre-grasp")) {
    fail(); return;
  }

  
  arm_group_->setMaxVelocityScalingFactor(0.1);
  arm_group_->setMaxAccelerationScalingFactor(0.1);
  if (!cart_move(arm_group_, td_pose(gx_pick, gy_pick, grasp_l8, shape_yaw), L, "Descend")) {
    fail(); return;
  }

  // Grip
  strong_grip(hand_group_, L);

  
  arm_group_->setMaxVelocityScalingFactor(0.5);
  arm_group_->setMaxAccelerationScalingFactor(0.5);
  if (!cart_move(arm_group_, td_pose(gx_pick, gy_pick, transit_l8, shape_yaw), L, "Lift")) {
    fail(); return;
  }

  // placing the object

  
  if (!joint_move(arm_group_, td_pose(gx, gy, transit_l8, 0.0), L, "Above basket")) {
    fail(); return;
  }

  
  if (!cart_move(arm_group_, td_pose(gx, gy, release_l8, 0.0), L, "Lower")) {
    fail(); return;
  }

  
  if (!open_gripper(hand_group_, L)) { fail(); return; }

  cart_move(arm_group_, td_pose(gx, gy, transit_l8, 0.0), L, "Retreat");
  go_home(arm_group_, L);
  RCLCPP_INFO(L, "=== Task 1 complete ===");
}

///////////////////////////////////////////////////////////////////////////////
// Task 2: Shape Detection (stub)
///////////////////////////////////////////////////////////////////////////////

void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  (void)request;
  response->mystery_object_num = -1;
  RCLCPP_WARN(node_->get_logger(), "Task 2 not yet implemented");
}

///////////////////////////////////////////////////////////////////////////////
// Task 3: Planning and Execution (stub)
///////////////////////////////////////////////////////////////////////////////

void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  (void)request;
  response->total_num_shapes = 0;
  response->num_most_common_shape = 0;
  RCLCPP_WARN(node_->get_logger(), "Task 3 not yet implemented");
}
