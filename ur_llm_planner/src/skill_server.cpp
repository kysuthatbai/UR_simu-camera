// skill_server.cpp
//
// Skill Server cho UR3/UR3e: cung cap service /execute_skill (ur_llm_planner/srv/ExecuteSkill).
// Moi skill duoc thuc hien HOAN TOAN bang MoveIt 2 (MoveGroupInterface):
//   - joint-space planning (OMPL) cho cac chuyen dong lon  -> kiem tra joint limit + collision
//   - Cartesian path (computeCartesianPath, avoid_collisions = true) cho buoc ha/nang thang dung
// LLM khong bao gio sinh quy dao / gia tri khop: no chi chon ten skill + tham so (object, zone).
//
// Skill quan sat (camera, KHONG di chuyen robot, tra ve data JSON):
//   detect_objects()      : liet ke moi vat, vi tri, vung chua vat
//   check_zone(zone)      : vung dang trong hay dang co vat nao
//   find_object(object)   : vi tri + vung cua mot vat
//   find_free_position()  : o trong tren ban (ngoai moi vung, robot voi toi) -> temporary_position
// Skill chuyen dong:
//   home, pick(object), place(object, zone | temporary_position), move_above(object),
//   move_to_zone(zone | temporary_position), open_gripper, close_gripper
//
// Trang thai moi truong lay tu CAMERA (perception_node -> /perception/scene):
//   - vi tri tung khoi, khoi nao dang nhin thay; khoi nao nam trong vung nao.
//   - Truoc moi skill, server cho mot khung hinh moi cua camera, dong bo planning scene cua
//     MoveIt theo vi tri quan sat duoc. Truoc khi gap, cho camera thay lai vat can gap.
//   - Khong co vi tri vat nao duoc khai bao san; chi vi tri cac vung la co dinh (scene.yaml).
//   - Ngay sau khi tha, vat nam duoi gripper (bi che) -> dung vi tri uoc luong luc tha
//     (source = estimated) cho toi khi camera thay lai vat.
//
// Gripper: Robotiq 2F-85 (urdf/ur_robotiq.urdf.xacro), diem tham chieu la grasp_tcp (tam kep).
//   - Ngon kep dong/mo bang gripper_controller (JointTrajectoryController, 6 khop theo he so mimic).
//   - close_gripper: ha grasp_tcp xuong tam vat, dong ngon toi gripper_grasp_position (khe ho
//     34 mm < khoi 40 mm -> ngon ep vao vat). Doc /joint_states: ngon bi vat chan lai (lech so voi lenh)
//     => da cham vat => mo ngon ve dung be mat vat (khe ho = cube_size + grasp_hold_clearance),
//     roi moi attach vat vao grasp_tcp trong MoveIt va khoa vat vao gripper trong Gazebo bang
//     DetachableJoint (topic /gripper/<vat>/attach). Sau khi khoa, Gazebo khong con tinh va cham
//     giua ngon va vat -> neu van giu lenh 34 mm thi ngon xuyen vao vat. Ngon dong het ma khong
//     cham gi => NO_OBJECT_TO_GRASP, khong khoa gi ca.
//   - open_gripper: ha vat xuong mat phang ben duoi, mo ngon, nha DetachableJoint, detach MoveIt.
//   - Khong bao gio dat truc tiep pose cua vat trong Gazebo: vat chi di chuyen do vat ly.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ur_llm_planner/srv/execute_skill.hpp>
#include <ur_llm_planner/msg/scene_observation.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using ExecuteSkill = ur_llm_planner::srv::ExecuteSkill;
using SceneObservation = ur_llm_planner::msg::SceneObservation;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

namespace status
{
constexpr const char * SUCCESS = "SUCCESS";
constexpr const char * FAILED = "FAILED";
constexpr const char * INVALID_SKILL = "INVALID_SKILL";
constexpr const char * INVALID_OBJECT = "INVALID_OBJECT";
constexpr const char * INVALID_ZONE = "INVALID_ZONE";
constexpr const char * PLANNING_FAILED = "PLANNING_FAILED";
constexpr const char * EXECUTION_FAILED = "EXECUTION_FAILED";
constexpr const char * OBJECT_NOT_HELD = "OBJECT_NOT_HELD";
constexpr const char * GRIPPER_BUSY = "GRIPPER_BUSY";
constexpr const char * NO_OBJECT_TO_GRASP = "NO_OBJECT_TO_GRASP";
constexpr const char * OBJECT_NOT_VISIBLE = "OBJECT_NOT_VISIBLE";
constexpr const char * ZONE_OCCUPIED = "ZONE_OCCUPIED";
constexpr const char * NO_FREE_SPACE = "NO_FREE_SPACE";
}  // namespace status

struct SkillResult
{
  SkillResult(std::string s = "", std::string m = "", std::string d = "")
  : status(std::move(s)), message(std::move(m)), data(std::move(d)) {}
  std::string status;
  std::string message;
  std::string data;  // ket qua dang JSON cua skill quan sat (camera), "" voi skill chuyen dong
  bool ok() const {return status == status::SUCCESS;}
};

static SkillResult success(const std::string & msg, const std::string & data = "")
{
  return {status::SUCCESS, msg, data};
}

// Dich cua place / move_to_zone: vi tri trong tren ban do find_free_position tim duoc.
static constexpr const char * kTempPosition = "temporary_position";

using Vec3 = std::array<double, 3>;

// Trang thai mot khoi, cap nhat tu camera.
struct ObjectState
{
  Vec3 position{};
  bool known = false;      // da co vi tri (camera da thay, hoac uoc luong luc tha)
  bool visible = false;    // camera thay trong khung hinh gan nhat
  bool estimated = false;  // vi tri uoc luong luc tha, cho camera xac nhan lai
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};  // thoi diem camera thay gan nhat
};

static std::string fmt(double v)
{
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(3);
  ss << v;
  return ss.str();
}

class SkillServer
{
public:
  explicit SkillServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    loadParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto latched = rclcpp::QoS(1).transient_local().reliable();
    state_pub_ = node_->create_publisher<std_msgs::msg::String>("scene_state", latched);
    held_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "skill_server/held_object", latched);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "skill_server/markers", latched);

    client_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    skill_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sensor_options;
    sensor_options.callback_group = sensor_group_;
    observation_sub_ = node_->create_subscription<SceneObservation>(
      perception_topic_, latched,
      [this](const SceneObservation::SharedPtr msg) {onObservation(msg);}, sensor_options);
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointState(msg);},
      sensor_options);
    state_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1), [this]() {publishSceneState();}, sensor_group_);

    for (const auto & kv : objects_) {
      gz_attach_pubs_[kv.first] = node_->create_publisher<std_msgs::msg::Empty>(
        "/gripper/" + kv.first + "/attach", 10);
      gz_detach_pubs_[kv.first] = node_->create_publisher<std_msgs::msg::Empty>(
        "/gripper/" + kv.first + "/detach", 10);
    }
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node_, gripper_action_, client_group_);
  }

  // Tao MoveGroupInterface (cho toi khi move_group san sang), dung planning scene, mo service.
  void initialize()
  {
    RCLCPP_INFO(node_->get_logger(), "Dang ket noi toi move_group (group '%s')...",
      planning_group_.c_str());
    move_group_ = std::make_shared<MoveGroupInterface>(node_, planning_group_, tf_buffer_);
    move_group_->setPoseReferenceFrame(world_frame_);
    move_group_->setEndEffectorLink(ee_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->startStateMonitor();
    // Scene cuc bo de loc nghiem IK: tu va cham (theo SRDF) + va cham voi ban.
    ik_check_scene_ = std::make_shared<planning_scene::PlanningScene>(
      move_group_->getRobotModel());

    setupPlanningScene();
    const SkillResult opened = commandGripper(gripper_open_position_, "mo gripper");
    if (!opened.ok()) {
      RCLCPP_WARN(node_->get_logger(), "%s", opened.message.c_str());
    }
    releaseGazeboObjects();
    publishHeld();

    RCLCPP_INFO(node_->get_logger(), "Dang cho du lieu tu camera (%s)...",
      perception_topic_.c_str());
    if (waitForObservation(startup_perception_timeout_)) {
      syncPlanningScene();
      logCameraView();
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Chua nhan duoc %s sau %.0f s (perception_node chua chay?). Cac skill can vi tri vat "
        "se tra ve OBJECT_NOT_VISIBLE cho toi khi camera hoat dong.",
        perception_topic_.c_str(), startup_perception_timeout_);
    }
    publishMarkers();
    publishSceneState();

    service_ = node_->create_service<ExecuteSkill>(
      "execute_skill",
      [this](const std::shared_ptr<ExecuteSkill::Request> req,
      std::shared_ptr<ExecuteSkill::Response> res) {handleRequest(req, res);},
      rmw_qos_profile_services_default, skill_group_);

    RCLCPP_INFO(node_->get_logger(),
      "Skill server san sang. Service: /execute_skill | quan sat: detect_objects, check_zone, "
      "find_object, find_free_position | chuyen dong: home, pick, place, move_above, "
      "move_to_zone, open_gripper, close_gripper");
  }

private:
  // ------------------------------------------------------------------ parameters
  void loadParameters()
  {
    planning_group_ = node_->declare_parameter<std::string>("planning_group", "ur_manipulator");
    ee_link_ = node_->declare_parameter<std::string>("ee_link", "grasp_tcp");
    world_frame_ = node_->declare_parameter<std::string>("world_frame", "world");
    use_detachable_joint_ = node_->declare_parameter<bool>("use_detachable_joint", true);

    velocity_scaling_ = node_->declare_parameter<double>("velocity_scaling", 0.3);
    acceleration_scaling_ = node_->declare_parameter<double>("acceleration_scaling", 0.3);
    planning_time_ = node_->declare_parameter<double>("planning_time", 5.0);
    planning_retries_ = node_->declare_parameter<int>("planning_retries", 3);
    eef_step_ = node_->declare_parameter<double>("eef_step", 0.005);
    min_cartesian_fraction_ = node_->declare_parameter<double>("min_cartesian_fraction", 0.98);

    approach_height_ = node_->declare_parameter<double>("approach_height", 0.12);
    place_clearance_ = node_->declare_parameter<double>("place_clearance", 0.003);

    // Camera
    perception_topic_ = node_->declare_parameter<std::string>(
      "perception_topic", "/perception/scene");
    perception_timeout_ = node_->declare_parameter<double>("perception_timeout", 3.0);
    startup_perception_timeout_ = node_->declare_parameter<double>(
      "startup_perception_timeout", 30.0);

    gripper_action_ = node_->declare_parameter<std::string>(
      "gripper_action", "/gripper_controller/follow_joint_trajectory");
    gripper_joints_ = node_->declare_parameter<std::vector<std::string>>(
      "gripper_joints", {"robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint",
        "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
        "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint"});
    // He so mimic trong robotiq_2f_85_macro.urdf.xacro, theo thu tu gripper_joints
    gripper_multipliers_ = node_->declare_parameter<std::vector<double>>(
      "gripper_mimic_multipliers", {1.0, -1.0, 1.0, -1.0, -1.0, 1.0});
    if (gripper_joints_.size() != gripper_multipliers_.size()) {
      throw std::runtime_error("gripper_joints va gripper_mimic_multipliers phai cung so phan tu");
    }
    gripper_open_position_ = node_->declare_parameter<double>("gripper_open_position", 0.0);
    // Khe ho giua 2 ma kep (do bang TF): 0.43 rad -> 42.0 mm, 0.45 -> 39.8 mm, 0.50 -> 34.3 mm.
    // Lenh 0.50 rad (nho hon khoi 40 mm ~3 mm moi ben) -> ngon ep vao vat va bi vat chan lai
    // o ~0.45 rad. Sai lech lenh/thuc te > grasp_contact_tolerance la dau hieu da cham vat;
    // kep hut (khong co vat) thi ngon toi dung 0.50 rad, sai lech ~0.002 rad.
    gripper_grasp_position_ = node_->declare_parameter<double>("gripper_grasp_position", 0.50);
    grasp_contact_tolerance_ = node_->declare_parameter<double>("grasp_contact_tolerance", 0.02);
    // Sau khi xac nhan cham vat: khe ho giu vat = cube_size + grasp_hold_clearance (mat dem ngon
    // vua cham be mat vat, khong xuyen vao).
    grasp_hold_clearance_ = node_->declare_parameter<double>("grasp_hold_clearance", 0.0005);
    gripper_motion_time_ = node_->declare_parameter<double>("gripper_motion_time", 1.0);
    gripper_hold_time_ = node_->declare_parameter<double>("gripper_hold_time", 0.3);
    touch_links_ = node_->declare_parameter<std::vector<std::string>>(
      "touch_links", {"grasp_tcp", "robotiq_85_base_link",
        "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
        "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
        "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
        "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"});

    // Tu the nghi: canh tay xoay ra phia sau robot (shoulder_pan ~ pi, vung x < 0 khong co vung /
    // khoi nao) -> khong che camera tren cao, camera thay tron ven cac vung khi robot cho lenh.
    home_joints_ = node_->declare_parameter<std::vector<double>>(
      "home_joints", {3.14, -1.5708, 1.5708, -1.5708, -1.5708, 0.0});

    const auto table_size = node_->declare_parameter<std::vector<double>>(
      "table_size", {0.90, 1.00, 0.75});
    const auto table_center = node_->declare_parameter<std::vector<double>>(
      "table_center", {0.15, 0.0, -0.375});
    table_size_ = toVec3(table_size, "table_size");
    table_center_ = toVec3(table_center, "table_center");
    table_padding_ = node_->declare_parameter<double>("table_padding", 0.01);

    cube_size_ = node_->declare_parameter<double>("cube_size", 0.04);
    zone_size_ = node_->declare_parameter<double>("zone_size", 0.08);

    // O trong tren ban cho find_free_position: luoi o trong vanh khan [min_radius, max_radius]
    // quanh de robot, cach tam vung va cac khoi khac mot khoang toi thieu.
    free_step_ = node_->declare_parameter<double>("free_space.step", 0.05);
    free_min_x_ = node_->declare_parameter<double>("free_space.min_x", 0.0);
    free_min_radius_ = node_->declare_parameter<double>("free_space.min_radius", 0.20);
    free_max_radius_ = node_->declare_parameter<double>("free_space.max_radius", 0.34);
    free_zone_clearance_ = node_->declare_parameter<double>("free_space.zone_clearance", 0.10);
    free_object_clearance_ = node_->declare_parameter<double>(
      "free_space.object_clearance", 0.09);
    free_max_attempts_ = node_->declare_parameter<int>("free_space.max_attempts", 10);

    // Chi ten vat duoc khai bao; vi tri do camera cung cap.
    const auto object_names = node_->declare_parameter<std::vector<std::string>>(
      "object_names", std::vector<std::string>{});
    for (const auto & name : object_names) {
      if (!name.empty()) {
        objects_[name] = ObjectState{};
      }
    }

    const auto zone_names = node_->declare_parameter<std::vector<std::string>>(
      "zone_names", std::vector<std::string>{});
    for (const auto & name : zone_names) {
      const auto p = node_->declare_parameter<std::vector<double>>(
        "zones." + name + ".position", std::vector<double>{});
      zones_[name] = toVec3(p, "zones." + name + ".position");
    }

    if (objects_.empty() || zones_.empty()) {
      throw std::runtime_error(
              "Chua khai bao object_names / zone_names. Hay nap config/scene.yaml cho node.");
    }
  }

  static Vec3 toVec3(const std::vector<double> & v, const std::string & name)
  {
    if (v.size() != 3) {
      throw std::runtime_error("Parameter '" + name + "' phai la mang 3 phan tu [x, y, z]");
    }
    return {v[0], v[1], v[2]};
  }

  // ------------------------------------------------------------------ camera
  void onObservation(const SceneObservation::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto & o : msg->objects) {
      auto it = objects_.find(o.name);
      if (it == objects_.end() || o.name == held_object_) {
        continue;  // vat dang cam: vi tri theo grasp_tcp, khong theo camera
      }
      ObjectState & s = it->second;
      s.visible = o.visible;
      // Chi ghi de khi camera dang thay vat; vat bi che giu vi tri gan nhat (hoac uoc luong).
      if (o.visible || (!s.known && o.known)) {
        s.position = {o.position.x, o.position.y, o.position.z};
        s.known = true;
        s.estimated = false;
        s.stamp = rclcpp::Time(o.last_seen, RCL_ROS_TIME);
      }
    }
    last_observation_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
    observed_ = true;
    observation_cv_.notify_all();
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  // Cho mot khung hinh camera chup SAU thoi diem goi ham.
  bool waitForObservation(double timeout_s)
  {
    const rclcpp::Time start = node_->now();
    std::unique_lock<std::mutex> lock(state_mutex_);
    return observation_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s),
             [&] {return observed_ && last_observation_ >= start;});
  }

  // Cho camera thay `name` trong mot khung hinh chup sau thoi diem goi ham.
  bool waitForVisible(const std::string & name, double timeout_s)
  {
    const rclcpp::Time start = node_->now();
    std::unique_lock<std::mutex> lock(state_mutex_);
    return observation_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s),
             [&] {
               const ObjectState & s = objects_.at(name);
               return s.visible && s.stamp >= start;
             });
  }

  std::map<std::string, ObjectState> snapshot()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return objects_;
  }

  void setHeld(const std::string & name)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      held_object_ = name;
    }
    publishHeld();
  }

  void publishHeld()
  {
    std_msgs::msg::String msg;
    msg.data = held_object_;
    held_pub_->publish(msg);
  }

  void logCameraView()
  {
    std::ostringstream ss;
    for (const auto & kv : snapshot()) {
      ss << "\n  " << kv.first << ": ";
      if (!kv.second.known) {
        ss << "camera chua thay";
        continue;
      }
      const std::string zone = zoneOf(kv.second.position);
      ss << "(" << fmt(kv.second.position[0]) << ", " << fmt(kv.second.position[1]) << ") "
         << (zone.empty() ? "tren ban" : "trong " + zone);
    }
    RCLCPP_INFO(node_->get_logger(), "Camera quan sat duoc:%s", ss.str().c_str());
  }

  // ------------------------------------------------------------------ planning scene
  moveit_msgs::msg::CollisionObject makeBox(
    const std::string & id, const Vec3 & center, const Vec3 & size) const
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = world_frame_;
    obj.id = id;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {size[0], size[1], size[2]};
    geometry_msgs::msg::Pose pose;
    pose.position.x = center[0];
    pose.position.y = center[1];
    pose.position.z = center[2];
    pose.orientation.w = 1.0;
    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    return obj;
  }

  void setupPlanningScene()
  {
    // Neu skill_server khoi dong lai trong khi move_group van chay, go cac vat con dang attach
    // va xoa vi tri cu cua cac khoi (se them lai theo camera).
    for (const auto & kv : planning_scene_.getAttachedObjects()) {
      move_group_->detachObject(kv.first);
    }
    std::vector<std::string> ids;
    for (const auto & kv : objects_) {
      ids.push_back(kv.first);
    }
    planning_scene_.removeCollisionObjects(ids);

    // Ban ha thap table_padding de de robot (dat tren mat ban) khong bi coi la va cham.
    Vec3 table_center = table_center_;
    table_center[2] -= table_padding_ / 2.0;
    Vec3 table_size = table_size_;
    table_size[2] -= table_padding_;
    const auto table = makeBox("table", table_center, table_size);
    ik_check_scene_->processCollisionObjectMsg(table);
    if (!planning_scene_.applyCollisionObject(table)) {
      RCLCPP_WARN(node_->get_logger(), "Khong the them ban vao planning scene");
    }
    setHeld("");
  }

  // Dua vi tri cac khoi (theo camera) vao planning scene de MoveIt tranh va cham.
  void syncPlanningScene()
  {
    std::vector<moveit_msgs::msg::CollisionObject> objs;
    for (const auto & kv : snapshot()) {
      if (kv.first == held_object_ || !kv.second.known) {
        continue;  // vat dang cam da duoc attach vao grasp_tcp
      }
      objs.push_back(makeBox(kv.first, kv.second.position, {cube_size_, cube_size_, cube_size_}));
    }
    if (!objs.empty() && !planning_scene_.applyCollisionObjects(objs)) {
      RCLCPP_WARN(node_->get_logger(), "Khong cap nhat duoc vi tri vat trong planning scene");
    }
  }

  // ------------------------------------------------------------------ Gazebo (DetachableJoint)
  // Khoa / nha vat vao gripper trong Gazebo (DetachableJoint, bridge bang ros_gz_bridge).
  // Day la rang buoc vat ly giua vat va co tay, KHONG dich chuyen vat.
  void setGazeboAttached(const std::string & name, bool attached)
  {
    if (!use_detachable_joint_) {
      return;
    }
    const auto & pub = attached ? gz_attach_pubs_.at(name) : gz_detach_pubs_.at(name);
    if (pub->get_subscription_count() == 0) {
      RCLCPP_WARN(node_->get_logger(),
        "Chua co bridge cho %s - Gazebo se khong %s vat", pub->get_topic_name(),
        attached ? "khoa" : "nha");
    }
    pub->publish(std_msgs::msg::Empty());
  }

  bool waitForGazeboBridges(std::chrono::seconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      bool ready = true;
      for (const auto & kv : gz_detach_pubs_) {
        ready = ready && kv.second->get_subscription_count() > 0 &&
          gz_attach_pubs_.at(kv.first)->get_subscription_count() > 0;
      }
      if (ready) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  // Plugin DetachableJoint tu khoa moi vat vao gripper luc khoi dong -> nha het. Cac khoi van
  // nam nguyen tai cho (khong dat lai vi tri).
  void releaseGazeboObjects()
  {
    if (!use_detachable_joint_) {
      return;
    }
    if (!waitForGazeboBridges(std::chrono::seconds(10))) {
      RCLCPP_WARN(node_->get_logger(),
        "Khong thay bridge /gripper/<vat>/attach|detach. Robot van chay duoc nhung vat trong "
        "Gazebo se khong duoc khoa vao gripper.");
    }
    for (const auto & kv : objects_) {
      setGazeboAttached(kv.first, false);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::optional<Vec3> toolPosition()
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(world_frame_, ee_link_, tf2::TimePointZero);
      return Vec3{tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.translation.z};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Khong lay duoc TF %s -> %s: %s", world_frame_.c_str(), ee_link_.c_str(), e.what());
      return std::nullopt;
    }
  }

  // ------------------------------------------------------------------ state helpers
  bool validObject(const std::string & name) const {return objects_.count(name) > 0;}
  bool validZone(const std::string & name) const {return zones_.count(name) > 0;}

  std::string zoneOf(const Vec3 & p) const
  {
    const double half = zone_size_ / 2.0 + 0.01;
    for (const auto & kv : zones_) {
      if (std::fabs(p[0] - kv.second[0]) <= half && std::fabs(p[1] - kv.second[1]) <= half) {
        return kv.first;
      }
    }
    return "";
  }

  // Vat dang nam trong vung (theo camera), bo qua `exclude` va vat dang cam. "" = vung trong.
  std::string occupantOf(const std::string & zone, const std::string & exclude)
  {
    for (const auto & kv : snapshot()) {
      if (kv.first != exclude && kv.first != held_object_ && kv.second.known &&
        zoneOf(kv.second.position) == zone)
      {
        return kv.first;
      }
    }
    return "";
  }

  geometry_msgs::msg::Pose toolDownPose(double x, double y, double z) const
  {
    // grasp_tcp (cung huong tool0) chi thang xuong mat ban (RPY = [pi, 0, -pi/2]), cung huong
    // voi tu the home de co tay khong phai xoay nhieu. 2 ngon kep khep theo truc y cua world.
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = -M_SQRT1_2;
    pose.orientation.y = M_SQRT1_2;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
  }

  void publishSceneState()
  {
    std::map<std::string, ObjectState> objects;
    std::string held;
    std::optional<Vec3> temp;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);  // goi tu ca timer lan thread service
      objects = objects_;
      held = held_object_;
      temp = temp_position_;
    }
    std::map<std::string, std::string> zone_state;
    std::ostringstream ss;
    ss << "{\"held_object\": " << (held.empty() ? "null" : "\"" + held + "\"");
    ss << ", \"source\": \"camera\", \"objects\": {";
    bool first = true;
    for (const auto & kv : objects) {
      const ObjectState & s = kv.second;
      const std::string zone = (kv.first == held || !s.known) ? "" : zoneOf(s.position);
      if (!zone.empty() && !zone_state.count(zone)) {
        zone_state[zone] = kv.first;
      }
      const char * source = kv.first == held ? "gripper" :
        (!s.known ? "unknown" : (s.estimated ? "estimated" : "camera"));
      ss << (first ? "" : ", ") << "\"" << kv.first << "\": {\"position\": ";
      if (s.known) {
        ss << "[" << fmt(s.position[0]) << ", " << fmt(s.position[1]) << ", " <<
          fmt(s.position[2]) << "]";
      } else {
        ss << "null";
      }
      ss << ", \"in_zone\": " << (zone.empty() ? "null" : "\"" + zone + "\"")
         << ", \"visible\": " << (s.visible ? "true" : "false")
         << ", \"source\": \"" << source << "\"}";
      first = false;
    }
    ss << "}, \"zones\": [";
    first = true;
    for (const auto & kv : zones_) {
      ss << (first ? "" : ", ") << "\"" << kv.first << "\"";
      first = false;
    }
    ss << "], \"zone_state\": {";
    first = true;
    for (const auto & kv : zones_) {
      const auto it = zone_state.find(kv.first);
      ss << (first ? "" : ", ") << "\"" << kv.first << "\": " <<
        (it == zone_state.end() ? "null" : "\"" + it->second + "\"");
      first = false;
    }
    ss << "}, \"temporary_position\": " << (temp ? vecJson(*temp) : "null");
    ss << "}";
    std_msgs::msg::String msg;
    msg.data = ss.str();
    state_pub_->publish(msg);
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto & kv : zones_) {
      visualization_msgs::msg::Marker pad;
      pad.header.frame_id = world_frame_;
      pad.ns = "zones";
      pad.id = id++;
      pad.type = visualization_msgs::msg::Marker::CUBE;
      pad.pose.position.x = kv.second[0];
      pad.pose.position.y = kv.second[1];
      pad.pose.position.z = kv.second[2] + 0.001;
      pad.pose.orientation.w = 1.0;
      pad.scale.x = zone_size_;
      pad.scale.y = zone_size_;
      pad.scale.z = 0.002;
      pad.color.g = 0.8;
      pad.color.b = 0.3;
      pad.color.a = 0.5;
      arr.markers.push_back(pad);

      visualization_msgs::msg::Marker label = pad;
      label.ns = "zone_labels";
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.text = kv.first;
      label.pose.position.z += 0.08;
      label.scale.z = 0.03;
      label.color.r = label.color.g = label.color.b = 1.0;
      label.color.a = 1.0;
      arr.markers.push_back(label);
    }
    // temporary_position (vang) - chi hien khi find_free_position vua tim duoc va chua dung
    visualization_msgs::msg::Marker temp;
    temp.header.frame_id = world_frame_;
    temp.ns = kTempPosition;
    temp.id = 0;
    temp.type = visualization_msgs::msg::Marker::CUBE;
    temp.pose.orientation.w = 1.0;
    visualization_msgs::msg::Marker temp_label = temp;
    temp_label.id = 1;
    temp_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    if (temp_position_) {
      temp.pose.position.x = (*temp_position_)[0];
      temp.pose.position.y = (*temp_position_)[1];
      temp.pose.position.z = 0.001;
      temp.scale.x = temp.scale.y = cube_size_ + 0.02;
      temp.scale.z = 0.002;
      temp.color.r = 1.0;
      temp.color.g = 0.85;
      temp.color.a = 0.7;
      temp_label.pose = temp.pose;
      temp_label.pose.position.z += 0.08;
      temp_label.text = kTempPosition;
      temp_label.scale.z = 0.025;
      temp_label.color.r = temp_label.color.g = 1.0;
      temp_label.color.a = 1.0;
    } else {
      temp.action = temp_label.action = visualization_msgs::msg::Marker::DELETE;
    }
    arr.markers.push_back(temp);
    arr.markers.push_back(temp_label);
    marker_pub_->publish(arr);
  }

  void setTempPosition(const std::optional<Vec3> & position)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      temp_position_ = position;
    }
    publishMarkers();
  }

  // ------------------------------------------------------------------ motion primitives
  // Khop continuous (wrist_3_joint cua UR): MoveIt chuan hoa goc ve [-pi, pi], nhung controller
  // trong Gazebo dung goc chua chuan hoa (vd: 4.78 rad sau khi quay qua moc pi). Quy dao MoveIt
  // gui di lech dung 2*pi so voi trang thai thuc -> controller huy (PATH_TOLERANCE_VIOLATED,
  // "Position Error: -6.283185"). Dich ca quy dao cua khop do di k*2*pi de diem dau trung voi
  // goc thuc tu /joint_states. Tu the robot khong doi (goc lech nhau dung mot vong).
  void alignContinuousJoints(moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    auto & jt = trajectory.joint_trajectory;
    if (jt.points.empty()) {
      return;
    }
    const auto model = move_group_->getRobotModel();
    std::lock_guard<std::mutex> lock(joint_mutex_);
    for (size_t j = 0; j < jt.joint_names.size(); ++j) {
      const auto * joint = dynamic_cast<const moveit::core::RevoluteJointModel *>(
        model->getJointModel(jt.joint_names[j]));
      const auto actual = joint_positions_.find(jt.joint_names[j]);
      if (!joint || !joint->isContinuous() || actual == joint_positions_.end()) {
        continue;
      }
      const double turns = std::round(
        (actual->second - jt.points.front().positions[j]) / (2.0 * M_PI));
      if (turns == 0.0) {
        continue;
      }
      for (auto & point : jt.points) {
        point.positions[j] += turns * 2.0 * M_PI;
      }
      RCLCPP_INFO(node_->get_logger(), "Dich quy dao %s %+.0f vong (goc thuc %.3f rad)",
        jt.joint_names[j].c_str(), turns, actual->second);
    }
  }

  SkillResult planAndExecute(const std::string & what)
  {
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= planning_retries_ && !planned; ++attempt) {
      move_group_->setStartStateToCurrentState();
      planned = static_cast<bool>(move_group_->plan(plan));
      if (!planned) {
        RCLCPP_WARN(node_->get_logger(), "Lap ke hoach '%s' that bai (lan %d/%d)",
          what.c_str(), attempt, planning_retries_);
      }
    }
    if (!planned) {
      return {status::PLANNING_FAILED, "MoveIt khong tim duoc quy dao hop le cho: " + what};
    }
    alignContinuousJoints(plan.trajectory_);
    if (!static_cast<bool>(move_group_->execute(plan))) {
      return {status::EXECUTION_FAILED, "Thuc thi quy dao that bai: " + what};
    }
    return success(what);
  }

  // Cau hinh khop "de chiu" cho thao tac tren ban: |pan| <= pi, elbow-up, co tay huong xuong.
  // Loai bo cac nghiem IK xoan (vd: pan = 4.4 rad) - van hop le nhung sau do khong the
  // ha/nang thang dung bang Cartesian path.
  static bool preferredConfiguration(const double * v)
  {
    return std::fabs(v[0]) <= M_PI &&   // shoulder_pan
           v[1] <= 0.0 && v[1] >= -M_PI &&  // shoulder_lift: canh tay tren huong len
           v[2] >= 0.0 && v[2] <= M_PI &&   // elbow
           v[4] <= 0.0 && v[4] >= -M_PI;    // wrist_2
  }

  // Nghiem IK hop le: cau hinh "de chiu" + khong tu va cham / khong cham ban.
  moveit::core::GroupStateValidityCallbackFn ikValidityCheck() const
  {
    return [this](moveit::core::RobotState * state, const moveit::core::JointModelGroup * group,
             const double * v) {
             if (!preferredConfiguration(v)) {
               return false;
             }
             state->setJointGroupPositions(group, v);
             state->update();
             return !ik_check_scene_->isStateColliding(*state);
           };
  }

  // Giai IK (co rang buoc cau hinh + khong va cham), sau do de OMPL lap ke hoach trong khong
  // gian khop toi nghiem do -> quy dao duoc kiem tra joint limit, self-collision va va cham moi
  // truong. Neu OMPL that bai (vd: nghiem cham vat tren ban) thi giai lai IK tu seed khac.
  SkillResult moveToPose(const geometry_msgs::msg::Pose & pose, const std::string & what)
  {
    const auto current = move_group_->getCurrentState(2.0);
    if (!current) {
      return {status::FAILED, "Khong doc duoc trang thai hien tai cua robot"};
    }
    const auto * jmg = current->getJointModelGroup(planning_group_);
    const auto check = ikValidityCheck();

    SkillResult result{status::PLANNING_FAILED,
      "Khong tim duoc nghiem IK hop le cho " + what +
      " (ngoai tam voi / vuot joint limit / va cham)"};
    moveit::core::RobotState state(*current);
    constexpr int kIkAttempts = 22;  // seed: trang thai hien tai, home, roi 20 seed ngau nhien
    for (int attempt = 0; attempt < kIkAttempts; ++attempt) {
      if (attempt == 0) {
        state = *current;
      } else if (attempt == 1) {
        state.setJointGroupPositions(jmg, home_joints_);
      } else {
        state.setToRandomPositions(jmg);
      }
      if (!state.setFromIK(jmg, pose, ee_link_, attempt < 2 ? 0.1 : 0.05, check)) {
        continue;
      }
      std::vector<double> joints;
      state.copyJointGroupPositions(jmg, joints);
      move_group_->setJointValueTarget(joints);
      result = planAndExecute(what);
      if (result.status != status::PLANNING_FAILED) {
        return result;  // SUCCESS hoac EXECUTION_FAILED: khong thu nghiem IK khac
      }
      RCLCPP_WARN(node_->get_logger(), "'%s': thu nghiem IK khac (lan %d)", what.c_str(),
        attempt + 1);
    }
    return result;
  }

  // Chuyen dong thang (Cartesian) - dung cho ha/nang theo phuong thang dung.
  SkillResult moveLinear(const geometry_msgs::msg::Pose & target, const std::string & what)
  {
    move_group_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_->computeCartesianPath(
      {target}, eef_step_, 0.0, trajectory, true);
    if (fraction < min_cartesian_fraction_) {
      std::ostringstream ss;
      ss << "Cartesian path '" << what << "' chi dat " << fraction * 100.0
         << "% (co the do va cham / vuot gioi han khop / IK that bai)";
      return {status::PLANNING_FAILED, ss.str()};
    }
    alignContinuousJoints(trajectory);
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if (!static_cast<bool>(move_group_->execute(plan))) {
      return {status::EXECUTION_FAILED, "Thuc thi Cartesian path that bai: " + what};
    }
    return success(what);
  }

  // Goc robotiq_85_left_knuckle_joint de khe ho giua 2 mat dem ngon bang `gap` (m).
  // Hinh hoc lay tu robotiq_2f_85_macro.urdf.xacro + mesh collision/left_finger_tip.stl:
  // mat dem ngon trai o x = kKnuckleX + kFingerX*cos(t) - kFingerZ*sin(t) + kPadX (frame
  // robotiq_85_base_link), finger_tip xoay nguoc lai nen mat dem luon song song.
  // Kiem tra: t = 0.45 -> 39.8 mm, t = 0.50 -> 34.3 mm (khop voi do bang TF).
  static double gripperAngleForGap(double gap)
  {
    constexpr double kKnuckleX = 0.03060114;              // left_knuckle_joint origin x
    constexpr double kFingerX = 0.03152616 + 0.00563134;  // finger_joint + finger_tip_joint x
    constexpr double kFingerZ = -0.00376347 + 0.04718515;  // finger_joint + finger_tip_joint z
    constexpr double kPadX = -0.02525865;                 // mat trong cua left_finger_tip mesh
    const double c = gap / 2.0 - kKnuckleX - kPadX;
    const double r = std::hypot(kFingerX, kFingerZ);
    return std::acos(std::clamp(c / r, -1.0, 1.0)) - std::atan2(kFingerZ, kFingerX);
  }

  // Dua ngon kep toi goc `position` cua robotiq_85_left_knuckle_joint (0 = mo, 0.8 = dong)
  // trong `duration` giay (<= 0: gripper_motion_time).
  SkillResult commandGripper(double position, const std::string & what, double duration = 0.0)
  {
    if (duration <= 0.0) {
      duration = gripper_motion_time_;
    }
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
      return {status::FAILED, "Action " + gripper_action_ + " chua san sang "
        "(gripper_controller chua chay?)"};
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = gripper_joints_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const double m : gripper_multipliers_) {
      point.positions.push_back(m * position);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(duration);
    goal.trajectory.points.push_back(point);

    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      return {status::EXECUTION_FAILED, "Khong gui duoc lenh toi gripper: " + what};
    }
    const auto handle = goal_future.get();
    if (!handle) {
      return {status::EXECUTION_FAILED, "gripper_controller tu choi lenh: " + what};
    }
    auto result_future = gripper_client_->async_get_result(handle);
    const auto timeout = std::chrono::duration<double>(duration + 5.0);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      return {status::EXECUTION_FAILED, "Het thoi gian cho gripper: " + what};
    }
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      return {status::EXECUTION_FAILED,
        "Gripper that bai (" + what + "): " + result.result->error_string};
    }
    return success(what);
  }

  // Sai lech lon nhat (rad) giua goc lenh va goc thuc cua cac khop ngon (tu /joint_states).
  // Ngon bi vat chan lai -> sai lech lon. Tra ve nullopt neu chua co du lieu khop.
  std::optional<double> gripperDeviation(double position)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    double deviation = 0.0;
    for (size_t i = 0; i < gripper_joints_.size(); ++i) {
      const auto it = joint_positions_.find(gripper_joints_[i]);
      if (it == joint_positions_.end()) {
        return std::nullopt;
      }
      deviation = std::max(deviation, std::fabs(it->second - gripper_multipliers_[i] * position));
    }
    return deviation;
  }

  // ------------------------------------------------------------------ skills
  SkillResult skillHome()
  {
    if (!move_group_->setJointValueTarget(home_joints_)) {
      return {status::FAILED, "home_joints khong hop le (sai so khop hoac vuot joint limit)"};
    }
    return planAndExecute("home");
  }

  // Vi tri moi nhat cua vat theo camera. Cho camera thay lai vat; neu vat dang bi che (vd: vua
  // duoc tha, gripper con o phia tren) thi dung vi tri quan sat / uoc luong gan nhat.
  std::optional<Vec3> observedPosition(const std::string & object, std::string & note)
  {
    if (waitForVisible(object, perception_timeout_)) {
      note = "camera";
    } else {
      note = "vi tri gan nhat (camera hien khong thay)";
    }
    const ObjectState s = snapshot().at(object);
    if (!s.known) {
      return std::nullopt;
    }
    return s.position;
  }

  SkillResult skillMoveAbove(const std::string & object)
  {
    if (!validObject(object)) {
      return {status::INVALID_OBJECT, "Vat khong ton tai: '" + object + "'"};
    }
    if (object == held_object_) {
      return {status::FAILED, "Khong the di toi phia tren '" + object + "' vi dang cam no"};
    }
    std::string note;
    const auto p = observedPosition(object, note);
    if (!p) {
      return {status::OBJECT_NOT_VISIBLE, "Camera khong thay '" + object + "'"};
    }
    RCLCPP_INFO(node_->get_logger(), "%s tai (%s, %s) theo %s", object.c_str(),
      fmt((*p)[0]).c_str(), fmt((*p)[1]).c_str(), note.c_str());
    return moveToPose(
      toolDownPose((*p)[0], (*p)[1], (*p)[2] + approach_height_), "move_above(" + object + ")");
  }

  // Vi tri dat vat cua mot dich: vung (scene.yaml) hoac temporary_position (find_free_position).
  std::optional<Vec3> targetOf(const std::string & zone, SkillResult & error) const
  {
    if (zone == kTempPosition) {
      if (!temp_position_) {
        error = {status::FAILED, "Chua co temporary_position - phai goi find_free_position truoc"};
        return std::nullopt;
      }
      return temp_position_;
    }
    if (!validZone(zone)) {
      error = {status::INVALID_ZONE, "Vung khong ton tai: '" + zone + "'"};
      return std::nullopt;
    }
    return zones_.at(zone);
  }

  SkillResult skillMoveToZone(const std::string & zone)
  {
    SkillResult error;
    const auto z = targetOf(zone, error);
    if (!z) {
      return error;
    }
    return moveToPose(
      toolDownPose((*z)[0], (*z)[1], (*z)[2] + cube_size_ / 2.0 + approach_height_),
      "move_to_zone(" + zone + ")");
  }

  // Mat phang nam ngay duoi (x, y): mat ban (z = 0) hoac dinh vat cao nhat ben duoi.
  double surfaceBelow(double x, double y, const std::string & exclude)
  {
    double surface = 0.0;
    for (const auto & kv : snapshot()) {
      if (kv.first == exclude || !kv.second.known) {
        continue;
      }
      const Vec3 & p = kv.second.position;
      if (std::fabs(p[0] - x) < cube_size_ && std::fabs(p[1] - y) < cube_size_) {
        surface = std::max(surface, p[2] + cube_size_ / 2.0);
      }
    }
    return surface;
  }

  // Ha grasp_tcp xuong tam vat ngay ben duoi, dong ngon, xac nhan cham vat, khoa vat, nang len.
  SkillResult skillCloseGripper(const std::string & expected = "")
  {
    if (!held_object_.empty()) {
      return {status::GRIPPER_BUSY, "Gripper dang giu '" + held_object_ + "'"};
    }
    const auto tool = toolPosition();
    if (!tool) {
      return {status::FAILED, "Khong xac dinh duoc vi tri " + ee_link_};
    }
    // Vat cao nhat nam thang hang ben duoi grasp_tcp (theo vi tri camera).
    std::string target;
    Vec3 p{};
    double best_z = -1e9;
    for (const auto & kv : snapshot()) {
      if (!kv.second.known) {
        continue;
      }
      const Vec3 & q = kv.second.position;
      const double dxy = std::hypot(q[0] - (*tool)[0], q[1] - (*tool)[1]);
      const double gap = (*tool)[2] - q[2];
      if (dxy < cube_size_ / 2.0 && gap > -0.005 && gap < approach_height_ + 0.03 &&
        q[2] > best_z)
      {
        target = kv.first;
        p = q;
        best_z = q[2];
      }
    }
    if (target.empty()) {
      return {status::NO_OBJECT_TO_GRASP, "Camera khong thay vat nao ngay duoi gripper"};
    }
    if (!expected.empty() && target != expected) {
      return {status::FAILED, "Vat ngay duoi gripper la '" + target + "', khong phai '" +
        expected + "' (co vat khac xep chong len tren?)"};
    }

    SkillResult r = commandGripper(gripper_open_position_, "mo gripper");
    if (!r.ok()) {return r;}
    r = moveLinear(toolDownPose(p[0], p[1], p[2]), "ha xuong gap " + target);
    if (!r.ok()) {return r;}
    r = commandGripper(gripper_grasp_position_, "kep " + target);
    if (!r.ok()) {return r;}

    // Xac nhan ngon da cham vat: ngon bi vat chan lai truoc khi toi goc lenh.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const auto deviation = gripperDeviation(gripper_grasp_position_);
    if (!deviation) {
      return {status::FAILED, "Chua co /joint_states cua cac khop ngon kep"};
    }
    RCLCPP_INFO(node_->get_logger(), "Kep %s: sai lech ngon %.4f rad (nguong %.4f)",
      target.c_str(), *deviation, grasp_contact_tolerance_);
    if (*deviation < grasp_contact_tolerance_) {
      commandGripper(gripper_open_position_, "mo gripper");
      moveLinear(toolDownPose(p[0], p[1], (*tool)[2]), "nang gripper len");
      return {status::NO_OBJECT_TO_GRASP,
        "Ngon kep dong het ma khong cham vat (" + target + " khong nam giua 2 ngon)"};
    }

    // Da cham vat: mo ngon ve dung be mat vat roi moi khoa. Sau khi khoa (DetachableJoint), vat
    // thuoc ve robot trong Gazebo va khong con va cham voi ngon -> giu lenh ep 34 mm thi ngon
    // se xuyen vao vat.
    const double hold = std::clamp(
      gripperAngleForGap(cube_size_ + grasp_hold_clearance_),
      gripper_open_position_, gripper_grasp_position_);
    r = commandGripper(hold, "giu " + target, gripper_hold_time_);
    if (!r.ok()) {return r;}
    RCLCPP_INFO(node_->get_logger(), "Giu %s: ngon %.3f rad (khe ho %.1f mm)", target.c_str(),
      hold, (cube_size_ + grasp_hold_clearance_) * 1000.0);

    if (!move_group_->attachObject(target, ee_link_, touch_links_)) {
      return {status::FAILED, "Khong attach duoc '" + target + "' vao " + ee_link_};
    }
    setGazeboAttached(target, true);
    setHeld(target);

    r = moveLinear(toolDownPose(p[0], p[1], (*tool)[2]), "nang " + target + " len");
    if (!r.ok()) {return r;}
    return success("Da gap '" + target + "'");
  }

  // Ha vat dang cam xuong mat phang ben duoi, mo ngon, nha vat, nang ve do cao cu.
  SkillResult skillOpenGripper()
  {
    if (held_object_.empty()) {
      const SkillResult r = commandGripper(gripper_open_position_, "mo gripper");
      return r.ok() ? success("Gripper da mo (khong giu vat nao)") : r;
    }
    const std::string object = held_object_;
    const auto tool = toolPosition();
    if (!tool) {
      return {status::FAILED, "Khong xac dinh duoc vi tri " + ee_link_};
    }
    const Vec3 drop{(*tool)[0], (*tool)[1], 0.0};
    const std::string zone = zoneOf(drop);
    if (!zone.empty()) {
      const std::string occupant = occupantOf(zone, object);
      if (!occupant.empty()) {
        return {status::ZONE_OCCUPIED, zone + " dang co '" + occupant + "' (camera)"};
      }
    }
    const double surface = surfaceBelow((*tool)[0], (*tool)[1], object);
    SkillResult r = moveLinear(
      toolDownPose((*tool)[0], (*tool)[1], surface + cube_size_ / 2.0 + place_clearance_),
      "ha " + object + " xuong");
    if (!r.ok()) {return r;}
    r = commandGripper(gripper_open_position_, "tha " + object);
    if (!r.ok()) {return r;}

    const auto release = toolPosition();
    setGazeboAttached(object, false);
    move_group_->detachObject(object);
    setHeld("");
    // Vat nam duoi gripper nen camera chua thay: uoc luong vi tri luc tha, cho camera xac nhan.
    const Vec3 at = release ? *release : *tool;
    Vec3 estimate{at[0], at[1], surface + cube_size_ / 2.0};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ObjectState & s = objects_.at(object);
      s.position = estimate;
      s.known = true;
      s.visible = false;
      s.estimated = true;
    }
    planning_scene_.applyCollisionObject(
      makeBox(object, estimate, {cube_size_, cube_size_, cube_size_}));
    // Tha dung tai temporary_position -> vi tri nay da co vat, lan sau phai tim vi tri moi.
    std::string where = zone.empty() ? " tren ban" : " vao " + zone;
    if (temp_position_ &&
      std::hypot(at[0] - (*temp_position_)[0], at[1] - (*temp_position_)[1]) < 0.01)
    {
      where = " tai " + std::string(kTempPosition);
      setTempPosition(std::nullopt);
    }

    r = moveLinear(toolDownPose((*tool)[0], (*tool)[1], (*tool)[2]), "nang gripper len");
    if (!r.ok()) {return r;}
    return success("Da tha '" + object + "'" + where);
  }

  SkillResult skillPick(const std::string & object)
  {
    if (!validObject(object)) {
      return {status::INVALID_OBJECT, "Vat khong ton tai: '" + object + "'"};
    }
    if (!held_object_.empty()) {
      return {status::GRIPPER_BUSY,
        "Dang giu '" + held_object_ + "', phai place truoc khi pick '" + object + "'"};
    }
    SkillResult r = skillMoveAbove(object);
    if (!r.ok()) {return r;}
    r = skillCloseGripper(object);
    if (!r.ok()) {return r;}
    return success("pick(" + object + ") thanh cong");
  }

  SkillResult skillPlace(const std::string & object, const std::string & zone)
  {
    if (!validObject(object)) {
      return {status::INVALID_OBJECT, "Vat khong ton tai: '" + object + "'"};
    }
    SkillResult error;
    const auto target = targetOf(zone, error);
    if (!target) {
      return error;
    }
    if (held_object_ != object) {
      return {status::OBJECT_NOT_HELD,
        "Gripper khong giu '" + object + "' (dang giu: '" +
        (held_object_.empty() ? "khong co gi" : held_object_) + "')"};
    }
    // Kiem tra dich bang camera truoc khi di chuyen.
    if (zone == kTempPosition) {
      const std::string blocker = objectNear(*target, free_object_clearance_, object);
      if (!blocker.empty()) {
        return {status::ZONE_OCCUPIED, std::string(kTempPosition) + " vua bi '" + blocker +
          "' chiem (camera) - goi lai find_free_position"};
      }
    } else {
      const std::string occupant = occupantOf(zone, object);
      if (!occupant.empty()) {
        return {status::ZONE_OCCUPIED,
          zone + " dang co '" + occupant + "' (camera) - phai chuyen '" + occupant +
          "' ra cho khac truoc"};
      }
    }
    SkillResult r = skillMoveToZone(zone);
    if (!r.ok()) {return r;}
    r = skillOpenGripper();
    if (!r.ok()) {return r;}
    return success("place(" + object + ", " + zone + ") thanh cong");
  }

  // Cac o trong tren ban (ngoai moi vung, cach cac khoi khac), gan gripper nhat truoc.
  std::vector<Vec3> freeTableCells(const std::string & exclude)
  {
    const auto objects = snapshot();
    const auto tool = toolPosition();
    const double xmin = std::max(free_min_x_, table_center_[0] - table_size_[0] / 2.0 + 0.05);
    const double xmax = table_center_[0] + table_size_[0] / 2.0 - 0.05;
    const double ymin = table_center_[1] - table_size_[1] / 2.0 + 0.05;
    const double ymax = table_center_[1] + table_size_[1] / 2.0 - 0.05;
    std::vector<std::pair<double, Vec3>> cells;
    for (double x = xmin; x <= xmax + 1e-9; x += free_step_) {
      for (double y = ymin; y <= ymax + 1e-9; y += free_step_) {
        const double r = std::hypot(x, y);
        if (r < free_min_radius_ || r > free_max_radius_) {
          continue;
        }
        bool free = true;
        for (const auto & z : zones_) {
          free = free && std::hypot(x - z.second[0], y - z.second[1]) >= free_zone_clearance_;
        }
        for (const auto & o : objects) {
          if (o.first != exclude && o.second.known) {
            free = free && std::hypot(x - o.second.position[0], y - o.second.position[1]) >=
              free_object_clearance_;
          }
        }
        if (free) {
          const double cost = tool ? std::hypot(x - (*tool)[0], y - (*tool)[1]) : r;
          cells.push_back({cost, Vec3{x, y, 0.0}});
        }
      }
    }
    std::sort(cells.begin(), cells.end(),
      [](const auto & a, const auto & b) {return a.first < b.first;});
    std::vector<Vec3> out;
    for (const auto & c : cells) {
      out.push_back(c.second);
    }
    return out;
  }

  // Giai IK cho pose (rang buoc cau hinh + khong va cham ban) ma KHONG di chuyen robot.
  bool reachable(const geometry_msgs::msg::Pose & pose)
  {
    const auto current = move_group_->getCurrentState(2.0);
    if (!current) {
      return false;
    }
    const auto * jmg = current->getJointModelGroup(planning_group_);
    const auto check = ikValidityCheck();
    moveit::core::RobotState state(*current);
    for (int attempt = 0; attempt < 8; ++attempt) {
      if (attempt == 1) {
        state.setJointGroupPositions(jmg, home_joints_);
      } else if (attempt > 1) {
        state.setToRandomPositions(jmg);
      }
      if (state.setFromIK(jmg, pose, ee_link_, 0.05, check)) {
        return true;
      }
    }
    return false;
  }

  // Vat da biet vi tri (khong phai `exclude` / vat dang cam) cach (x, y) cua p duoi `radius`.
  std::string objectNear(const Vec3 & p, double radius, const std::string & exclude)
  {
    for (const auto & kv : snapshot()) {
      if (kv.first != exclude && kv.first != held_object_ && kv.second.known &&
        std::hypot(kv.second.position[0] - p[0], kv.second.position[1] - p[1]) < radius)
      {
        return kv.first;
      }
    }
    return "";
  }

  static std::string vecJson(const Vec3 & p)
  {
    return "[" + fmt(p[0]) + ", " + fmt(p[1]) + ", " + fmt(p[2]) + "]";
  }

  // ------------------------------------------------------------------ skill quan sat (camera)
  // Cac skill nay khong di chuyen robot: chi doc khung hinh camera moi va tra ve ket qua
  // (message cho nguoi doc + data JSON cho llm_planner_node kiem tra).

  // detect_objects(): liet ke moi vat camera thay va vung chua vat.
  SkillResult skillDetectObjects()
  {
    if (!waitForObservation(perception_timeout_)) {
      return {status::FAILED, "Camera khong gui khung hinh moi"};
    }
    std::ostringstream msg, data;
    data << "{\"objects\": {";
    bool first = true;
    for (const auto & kv : snapshot()) {
      const ObjectState & s = kv.second;
      const bool held = kv.first == held_object_;
      const std::string zone = (held || !s.known) ? "" : zoneOf(s.position);
      msg << (first ? "" : ", ") << kv.first << ": " <<
      (held ? "dang cam" : (!s.known ? "chua thay" : (zone.empty() ? "tren ban" : zone)));
      data << (first ? "" : ", ") << "\"" << kv.first << "\": {\"position\": " <<
      (s.known ? vecJson(s.position) : "null") << ", \"in_zone\": " <<
      (zone.empty() ? "null" : "\"" + zone + "\"") << ", \"visible\": " <<
      (s.visible ? "true" : "false") << ", \"held\": " << (held ? "true" : "false") << "}";
      first = false;
    }
    data << "}}";
    return success(msg.str(), data.str());
  }

  // check_zone(zone): camera xac nhan vung dang trong hay dang co vat nao.
  SkillResult skillCheckZone(const std::string & zone)
  {
    if (!validZone(zone)) {
      return {status::INVALID_ZONE, "Vung khong ton tai: '" + zone + "'"};
    }
    if (!waitForObservation(perception_timeout_)) {
      return {status::FAILED, "Camera khong gui khung hinh moi"};
    }
    const std::string occupant = occupantOf(zone, "");
    const std::string data = "{\"zone\": \"" + zone + "\", \"occupied\": " +
      (occupant.empty() ? "false" : "true") + ", \"object\": " +
      (occupant.empty() ? "null" : "\"" + occupant + "\"") + "}";
    return success(zone + (occupant.empty() ? ": trong" : ": dang co " + occupant) +
             " (camera)", data);
  }

  // find_object(object): camera tim vat, tra ve vi tri va vung chua vat.
  SkillResult skillFindObject(const std::string & object)
  {
    if (!validObject(object)) {
      return {status::INVALID_OBJECT, "Vat khong ton tai: '" + object + "'"};
    }
    if (object == held_object_) {
      return success(object + ": dang nam trong gripper",
               "{\"object\": \"" + object + "\", \"held\": true}");
    }
    std::string note;
    const auto p = observedPosition(object, note);
    if (!p) {
      return {status::OBJECT_NOT_VISIBLE, "Camera khong thay '" + object + "'"};
    }
    const std::string zone = zoneOf(*p);
    const std::string data = "{\"object\": \"" + object + "\", \"position\": " + vecJson(*p) +
      ", \"in_zone\": " + (zone.empty() ? "null" : "\"" + zone + "\"") + ", \"visible\": " +
      (note == "camera" ? "true" : "false") + ", \"held\": false}";
    return success(object + " tai (" + fmt((*p)[0]) + ", " + fmt((*p)[1]) + ") " +
             (zone.empty() ? "tren ban" : "trong " + zone) + " theo " + note, data);
  }

  // find_free_position(): tim o trong tren ban (ngoai moi vung, cach cac vat khac - theo camera)
  // ma robot voi toi duoc (giai IK, khong di chuyen), luu lam temporary_position.
  SkillResult skillFindFreePosition()
  {
    if (!waitForObservation(perception_timeout_)) {
      RCLCPP_WARN(node_->get_logger(), "Khong co khung hinh camera moi - dung trang thai cu");
    }
    const auto cells = freeTableCells(held_object_);
    int checked = 0;
    for (const Vec3 & c : cells) {
      if (checked++ >= free_max_attempts_) {
        break;
      }
      const std::string xy = "(" + fmt(c[0]) + ", " + fmt(c[1]) + ")";
      if (!reachable(toolDownPose(c[0], c[1], cube_size_ / 2.0 + approach_height_)) ||
        !reachable(toolDownPose(c[0], c[1], cube_size_ / 2.0 + place_clearance_)))
      {
        RCLCPP_INFO(node_->get_logger(), "O trong %s ngoai tam voi - thu o khac", xy.c_str());
        continue;
      }
      setTempPosition(Vec3{c[0], c[1], 0.0});
      return success(std::string(kTempPosition) + " = " + xy + " (o trong theo camera, " +
               std::to_string(cells.size()) + " o ung vien)",
               "{\"position\": " + vecJson(Vec3{c[0], c[1], 0.0}) + "}");
    }
    return {status::NO_FREE_SPACE, cells.empty() ?
      "Camera khong thay o trong nao tren ban" : "Khong co o trong nao trong tam voi"};
  }

  // ------------------------------------------------------------------ service
  SkillResult dispatch(const std::string & skill, const std::string & object,
    const std::string & zone)
  {
    if (skill == "home") {return skillHome();}
    if (skill == "pick") {return skillPick(object);}
    if (skill == "place") {return skillPlace(object, zone);}
    if (skill == "detect_objects") {return skillDetectObjects();}
    if (skill == "check_zone") {return skillCheckZone(zone);}
    if (skill == "find_object") {return skillFindObject(object);}
    if (skill == "find_free_position") {return skillFindFreePosition();}
    if (skill == "move_above") {return skillMoveAbove(object);}
    if (skill == "move_to_zone") {return skillMoveToZone(zone);}
    if (skill == "open_gripper") {return skillOpenGripper();}
    if (skill == "close_gripper") {return skillCloseGripper();}
    return {status::INVALID_SKILL, "Skill khong nam trong danh sach cho phep: '" + skill + "'"};
  }

  void handleRequest(
    const std::shared_ptr<ExecuteSkill::Request> req,
    std::shared_ptr<ExecuteSkill::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(), ">> skill=%s object=%s zone=%s",
      req->skill.c_str(), req->object.c_str(), req->zone.c_str());
    // Kiem tra moi truong truoc moi skill: lay khung hinh camera moi, cap nhat planning scene.
    if (!waitForObservation(perception_timeout_)) {
      RCLCPP_WARN(node_->get_logger(), "Khong co khung hinh camera moi - dung trang thai cu");
    }
    syncPlanningScene();

    SkillResult r;
    try {
      r = dispatch(req->skill, req->object, req->zone);
    } catch (const std::exception & e) {
      r = {status::FAILED, std::string("Exception: ") + e.what()};
    }
    // Vat dang cam nam giua 2 ngon: tam vat trung voi grasp_tcp.
    if (!held_object_.empty()) {
      if (const auto tool = toolPosition()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        objects_[held_object_].position = *tool;
        objects_[held_object_].known = true;
      }
    }
    publishSceneState();
    res->status = r.status;
    res->message = r.message;
    res->data = r.data;
    if (r.ok()) {
      RCLCPP_INFO(node_->get_logger(), "<< %s: %s", r.status.c_str(), r.message.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "<< %s: %s", r.status.c_str(), r.message.c_str());
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  planning_scene::PlanningScenePtr ik_check_scene_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::CallbackGroup::SharedPtr skill_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp::Service<ExecuteSkill>::SharedPtr service_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp::Subscription<SceneObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> gz_attach_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> gz_detach_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr held_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string planning_group_, ee_link_, world_frame_;
  bool use_detachable_joint_ = true;
  double velocity_scaling_ = 0.3, acceleration_scaling_ = 0.3, planning_time_ = 5.0;
  int planning_retries_ = 3;
  double eef_step_ = 0.005, min_cartesian_fraction_ = 0.98;
  double approach_height_ = 0.12, place_clearance_ = 0.003;
  std::string perception_topic_;
  double perception_timeout_ = 3.0, startup_perception_timeout_ = 30.0;
  std::vector<double> home_joints_;
  Vec3 table_size_{}, table_center_{};
  double table_padding_ = 0.01;
  double cube_size_ = 0.04, zone_size_ = 0.08;
  double free_step_ = 0.05, free_min_x_ = 0.0, free_min_radius_ = 0.20, free_max_radius_ = 0.34;
  double free_zone_clearance_ = 0.10, free_object_clearance_ = 0.09;
  int free_max_attempts_ = 10;

  std::string gripper_action_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> gripper_multipliers_;
  double gripper_open_position_ = 0.0, gripper_grasp_position_ = 0.50, gripper_motion_time_ = 1.0;
  double gripper_hold_time_ = 0.3;
  double grasp_contact_tolerance_ = 0.02, grasp_hold_clearance_ = 0.0005;
  std::vector<std::string> touch_links_;

  // Trang thai moi truong (camera) - dung chung giua thread service va thread camera.
  std::mutex state_mutex_;
  std::condition_variable observation_cv_;
  std::map<std::string, ObjectState> objects_;
  std::string held_object_;  // chi thread service ghi (duoi state_mutex_)
  rclcpp::Time last_observation_{0, 0, RCL_ROS_TIME};
  bool observed_ = false;
  std::map<std::string, Vec3> zones_;
  std::optional<Vec3> temp_position_;  // ket qua find_free_position gan nhat (chua dung)

  std::mutex joint_mutex_;
  std::map<std::string, double> joint_positions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("skill_server");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int rc = 0;
  try {
    SkillServer server(node);
    server.initialize();
    spinner.join();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
    rc = 1;
    rclcpp::shutdown();
    spinner.join();
  }
  rclcpp::shutdown();
  return rc;
}
