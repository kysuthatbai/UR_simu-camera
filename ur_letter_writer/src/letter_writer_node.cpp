// letter_writer_node.cpp
//
// Dieu khien UR3/UR3e ve chu cai dau tien trong ten sinh vien bang
// MoveIt 2 (Cartesian path planning) tren mot mat phang Cartesian tuy chon.
//
// Y tuong:
//  - Moi chu cai duoc dinh nghia bang mot hoac nhieu "net" (stroke).
//  - Moi net la mot day diem (u, v) trong khong gian chuan hoa [0,1] x [0,1].
//  - (u, v) duoc anh xa sang toa do Cartesian thuc te thong qua:
//      mat_phang = { plane_center, u_axis, v_axis }  (normal = u_axis x v_axis)
//  - Giua hai net, dau cong tac duoc "nhac" len (lui ra khoi mat phang theo
//    huong normal) roi di chuyen sang vi tri bat dau cua net tiep theo.
//  - Toan bo duong di (tru buoc tiep can dau tien) duoc lap ke hoach bang
//    move_group.computeCartesianPath() de dam bao lien tuc va tranh self-collision.

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <map>
#include <vector>
#include <string>
#include <cctype>
#include <thread>
#include <mutex>
#include <chrono>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Dinh nghia font chu don gian: luoi 3x3 diem chuan (block letter, net thang)
// ---------------------------------------------------------------------------
struct Point2D
{
  double u;
  double v;
};
using Stroke = std::vector<Point2D>;
using LetterShape = std::vector<Stroke>;

namespace
{
const Point2D TL{0.0, 1.0}, TM{0.5, 1.0}, TR{1.0, 1.0};
const Point2D ML{0.0, 0.5}, MM{0.5, 0.5}, MR{1.0, 0.5};
const Point2D BL{0.0, 0.0}, BM{0.5, 0.0}, BR{1.0, 0.0};

// Tra ve thu vien chu cai A-Z. Co the mo rong / chinh sua tai day.
std::map<char, LetterShape> buildLetterLibrary()
{
  std::map<char, LetterShape> lib;
  lib['A'] = {{BL, TM, BR}, {ML, MR}};
  lib['B'] = {{BL, TL}, {TL, TR, MR, ML}, {ML, BR, BL}};
  lib['C'] = {{TR, TL, BL, BR}};
  lib['D'] = {{BL, TL}, {TL, TR, MR, BR, BL}};
  lib['E'] = {{TR, TL, BL, BR}, {ML, MR}};
  lib['F'] = {{BL, TL, TR}, {ML, MR}};
  lib['G'] = {{TR, TL, BL, BR}, {MR, MM}};
  lib['H'] = {{TL, BL}, {TR, BR}, {ML, MR}};
  lib['I'] = {{TM, BM}};
  lib['J'] = {{TR, BR, BL}};
  lib['K'] = {{TL, BL}, {TR, ML, BR}};
  lib['L'] = {{TL, BL, BR}};
  lib['M'] = {{BL, TL, MM, TR, BR}};
  lib['N'] = {{BL, TL, BR, TR}};
  lib['O'] = {{TL, TR, BR, BL, TL}};
  lib['P'] = {{BL, TL, TR, MR, ML}};
  lib['Q'] = {{TL, TR, BR, BL, TL}, {MM, BR}};
  lib['R'] = {{BL, TL, TR, MR, ML}, {ML, BR}};
  lib['S'] = {{
    {0.85, 0.90}, {0.50, 1.00}, {0.15, 0.90},
    {0.10, 0.72}, {0.25, 0.58}, {0.50, 0.50},
    {0.75, 0.42}, {0.90, 0.28}, {0.85, 0.10},
    {0.50, 0.00}, {0.15, 0.10}
  }};
  lib['T'] = {{TL, TR}, {TM, BM}};
  lib['U'] = {{TL, BL, BR, TR}};
  lib['V'] = {{TL, BM, TR}};
  lib['W'] = {{TL, BL, MM, BR, TR}};
  lib['X'] = {{TL, BR}, {TR, BL}};
  lib['Y'] = {{TL, MM}, {TR, MM}, {MM, BM}};
  lib['Z'] = {{TL, TR, BL, BR}};
  return lib;
}
}  // namespace

// Mot diem trong day chuyen dong day du cua chu (da bao gom cac lan nhac but)
struct SequencePoint
{
  Point2D pt;
  bool retracted;  // true = dang nhac but (di chuyen), false = dang cham (dang ve)
};

// ---------------------------------------------------------------------------
// LetterWriterNode
// ---------------------------------------------------------------------------
class LetterWriterNode
{
public:
  explicit LetterWriterNode(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    declareParameters();
    readParameters();
    buildPlaneBasis();

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "letter_writer/markers", rclcpp::QoS(10).transient_local());
  }

  bool run()
  {
    const char letter = resolveLetter();
    const auto lib = buildLetterLibrary();
    const auto it = lib.find(letter);
    if (it == lib.end()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Khong tim thay dinh nghia cho chu '%c'. Cac chu duoc ho tro: A-Z. "
        "Hay dat tham so 'letter' hoac sua 'student_name'.",
        letter);
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Chuan bi ve chu '%c' (nhom lap ke hoach: %s)",
      letter, planning_group_.c_str());

    moveit::planning_interface::MoveGroupInterface move_group(node_, planning_group_);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling_);
    planning_frame_ = move_group.getPlanningFrame();
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", planning_frame_.c_str());

    const LetterShape & shape = it->second;

    // Xay dung day (point, retracted) day du cho toan bo chu, bao gom
    // cac lan "nhac but" giua cac net.
    std::vector<SequencePoint> sequence;
    for (size_t s = 0; s < shape.size(); ++s) {
      const Stroke & stroke = shape[s];
      if (stroke.empty()) {continue;}
      if (s == 0) {
        sequence.push_back({stroke.front(), true});    // vi tri tiep can (nhac but)
        sequence.push_back({stroke.front(), false});   // ha but xuong diem dau
      } else {
        sequence.push_back({sequence.back().pt, true});   // nhac but tai diem cuoi net truoc
        sequence.push_back({stroke.front(), true});       // di chuyen (van nhac) den net moi
        sequence.push_back({stroke.front(), false});      // ha but xuong
      }
      for (size_t p = 1; p < stroke.size(); ++p) {
        sequence.push_back({stroke[p], false});
      }
    }
    if (sequence.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Dinh nghia chu rong, khong the ve.");
      return false;
    }
    sequence.push_back({sequence.back().pt, true});  // nhac but sau khi ve xong

    // Luu lai de publish lien tuc (moi giay) cho toi khi node bi dung (Ctrl+C),
    // dam bao RViz them Display tre van thay duoc duong di.
    {
      std::lock_guard<std::mutex> lock(sequence_mutex_);
      sequence_ = sequence;
    }
    publishMarkers();  // publish ngay lan dau
    marker_timer_ = node_->create_wall_timer(1s, [this]() {publishMarkers();});

    // ---- Giai doan 1: di chuyen (joint-space) den vi tri tiep can dau tien ----
    const geometry_msgs::msg::Pose pre_pose = poseAt(sequence.front().pt, true);
    move_group.setPoseTarget(pre_pose);
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    if (move_group.plan(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(),
        "Lap ke hoach den vi tri tiep can that bai. Kiem tra lai plane_center / "
        "gioi han khop cua %s.", planning_group_.c_str());
      return false;
    }
    if (move_group.execute(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Thuc thi di chuyen tiep can that bai.");
      return false;
    }

    // ---- Giai doan 2: Cartesian path cho toan bo net chu + nhac but cuoi ----
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(sequence.size() - 1);
    for (size_t i = 1; i < sequence.size(); ++i) {
      waypoints.push_back(poseAt(sequence[i].pt, sequence[i].retracted));
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;  // 0.0 = tat kiem tra "buoc nhay" khop (khuyen nghi)
    const double fraction = move_group.computeCartesianPath(
      waypoints, eef_step_, jump_threshold, trajectory, true);
    RCLCPP_INFO(node_->get_logger(), "Cartesian path dat duoc: %.1f%%", fraction * 100.0);

    if (fraction < min_fraction_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Fraction (%.2f) thap hon nguong yeu cau (%.2f) -> co the vuot gioi han khop "
        "hoac IK that bai o mot so diem. Hay giam letter_width/letter_height, doi "
        "plane_center gan robot hon, hoac doi truc u/v.",
        fraction, min_fraction_);
      return false;
    }

    if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Thuc thi quy dao ve chu that bai.");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Da ve xong chu '%c' thanh cong!", letter);
    RCLCPP_INFO(node_->get_logger(),
      "Duong di dang duoc publish lien tuc tren topic 'letter_writer/markers' "
      "(MarkerArray). Mo RViz -> Add -> By topic -> chon 'letter_writer/markers' "
      "-> MarkerArray de xem hinh chu. Nhan Ctrl+C tai terminal nay khi xem xong.");
    return true;
  }

private:
  void declareParameters()
  {
    node_->declare_parameter<std::string>("student_name", "Bao");
    node_->declare_parameter<std::string>("letter", "");
    node_->declare_parameter<std::string>("planning_group", "ur_manipulator");

    node_->declare_parameter<double>("plane_center_x", 0.35);
    node_->declare_parameter<double>("plane_center_y", 0.0);
    node_->declare_parameter<double>("plane_center_z", 0.30);

    node_->declare_parameter<double>("plane_u_axis_x", 0.0);
    node_->declare_parameter<double>("plane_u_axis_y", 1.0);
    node_->declare_parameter<double>("plane_u_axis_z", 0.0);

    node_->declare_parameter<double>("plane_v_axis_x", 0.0);
    node_->declare_parameter<double>("plane_v_axis_y", 0.0);
    node_->declare_parameter<double>("plane_v_axis_z", 1.0);

    node_->declare_parameter<double>("letter_width", 0.18);
    node_->declare_parameter<double>("letter_height", 0.22);
    node_->declare_parameter<double>("retract_distance", 0.05);

    node_->declare_parameter<double>("eef_step", 0.005);
    node_->declare_parameter<double>("min_fraction", 0.95);

    node_->declare_parameter<double>("velocity_scaling", 0.2);
    node_->declare_parameter<double>("acceleration_scaling", 0.2);
  }

  void readParameters()
  {
    student_name_ = node_->get_parameter("student_name").as_string();
    letter_param_ = node_->get_parameter("letter").as_string();
    planning_group_ = node_->get_parameter("planning_group").as_string();

    plane_center_ = tf2::Vector3(
      node_->get_parameter("plane_center_x").as_double(),
      node_->get_parameter("plane_center_y").as_double(),
      node_->get_parameter("plane_center_z").as_double());

    raw_u_axis_ = tf2::Vector3(
      node_->get_parameter("plane_u_axis_x").as_double(),
      node_->get_parameter("plane_u_axis_y").as_double(),
      node_->get_parameter("plane_u_axis_z").as_double());

    raw_v_axis_ = tf2::Vector3(
      node_->get_parameter("plane_v_axis_x").as_double(),
      node_->get_parameter("plane_v_axis_y").as_double(),
      node_->get_parameter("plane_v_axis_z").as_double());

    letter_width_ = node_->get_parameter("letter_width").as_double();
    letter_height_ = node_->get_parameter("letter_height").as_double();
    retract_distance_ = node_->get_parameter("retract_distance").as_double();

    eef_step_ = node_->get_parameter("eef_step").as_double();
    min_fraction_ = node_->get_parameter("min_fraction").as_double();

    velocity_scaling_ = node_->get_parameter("velocity_scaling").as_double();
    acceleration_scaling_ = node_->get_parameter("acceleration_scaling").as_double();
  }

  // Xay dung he truc (u_axis, v_axis, normal_axis) truc chuan-giao (Gram-Schmidt)
  // tu 2 truc do nguoi dung nhap, roi tinh quaternion co dinh cho dau cong tac.
  void buildPlaneBasis()
  {
    u_axis_ = raw_u_axis_.normalized();
    tf2::Vector3 n = u_axis_.cross(raw_v_axis_);
    normal_axis_ = n.normalized();
    v_axis_ = normal_axis_.cross(u_axis_).normalized();

    // Cot cua ma tran xoay = (u_axis, v_axis, normal_axis) trong he base_link
    tf2::Matrix3x3 basis(
      u_axis_.x(), v_axis_.x(), normal_axis_.x(),
      u_axis_.y(), v_axis_.y(), normal_axis_.y(),
      u_axis_.z(), v_axis_.z(), normal_axis_.z());
    tf2::Quaternion q;
    basis.getRotation(q);
    q.normalize();
    fixed_orientation_.x = q.x();
    fixed_orientation_.y = q.y();
    fixed_orientation_.z = q.z();
    fixed_orientation_.w = q.w();
  }

  char resolveLetter() const
  {
    if (!letter_param_.empty()) {
      return static_cast<char>(std::toupper(static_cast<unsigned char>(letter_param_[0])));
    }
    for (unsigned char c : student_name_) {
      if (std::isalpha(c)) {
        return static_cast<char>(std::toupper(c));
      }
    }
    return '?';
  }

  geometry_msgs::msg::Pose poseAt(const Point2D & p, bool retracted) const
  {
    const double offset_u = (p.u - 0.5) * letter_width_;
    const double offset_v = (p.v - 0.5) * letter_height_;
    tf2::Vector3 pos = plane_center_ + u_axis_ * offset_u + v_axis_ * offset_v;
    if (retracted) {
      pos -= normal_axis_ * retract_distance_;
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = pos.z();
    pose.orientation = fixed_orientation_;
    return pose;
  }

  // Publish 2 marker tach biet:
  //  - "letter_writer_draw" (mau xanh duong, net day): doan dau cong tac THUC SU
  //     cham vao mat phang (retracted=false o ca 2 dau) -> chinh la hinh chu.
  //  - "letter_writer_lift" (mau xam, mo): doan di chuyen luc nhac but giua cac net.
  // Duoc goi lai moi giay (bang wall timer) de RViz them Display tre van thay duoc.
  void publishMarkers()
  {
    std::vector<SequencePoint> sequence;
    {
      std::lock_guard<std::mutex> lock(sequence_mutex_);
      sequence = sequence_;
    }
    if (sequence.size() < 2) {return;}

    const std::string frame = planning_frame_.empty() ? "base_link" : planning_frame_;
    const rclcpp::Time stamp = node_->now();

    visualization_msgs::msg::Marker draw_marker;
    draw_marker.header.frame_id = frame;
    draw_marker.header.stamp = stamp;
    draw_marker.ns = "letter_writer_draw";
    draw_marker.id = 0;
    draw_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    draw_marker.action = visualization_msgs::msg::Marker::ADD;
    draw_marker.scale.x = 0.006;
    draw_marker.color.r = 0.1;
    draw_marker.color.g = 0.5;
    draw_marker.color.b = 1.0;
    draw_marker.color.a = 1.0;
    draw_marker.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker lift_marker;
    lift_marker.header.frame_id = frame;
    lift_marker.header.stamp = stamp;
    lift_marker.ns = "letter_writer_lift";
    lift_marker.id = 1;
    lift_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    lift_marker.action = visualization_msgs::msg::Marker::ADD;
    lift_marker.scale.x = 0.002;
    lift_marker.color.r = 0.6;
    lift_marker.color.g = 0.6;
    lift_marker.color.b = 0.6;
    lift_marker.color.a = 0.4;
    lift_marker.pose.orientation.w = 1.0;

    for (size_t i = 1; i < sequence.size(); ++i) {
      const geometry_msgs::msg::Pose p0 = poseAt(sequence[i - 1].pt, sequence[i - 1].retracted);
      const geometry_msgs::msg::Pose p1 = poseAt(sequence[i].pt, sequence[i].retracted);
      geometry_msgs::msg::Point a;
      a.x = p0.position.x; a.y = p0.position.y; a.z = p0.position.z;
      geometry_msgs::msg::Point b;
      b.x = p1.position.x; b.y = p1.position.y; b.z = p1.position.z;

      const bool is_draw_segment = !sequence[i - 1].retracted && !sequence[i].retracted;
      if (is_draw_segment) {
        draw_marker.points.push_back(a);
        draw_marker.points.push_back(b);
      } else {
        lift_marker.points.push_back(a);
        lift_marker.points.push_back(b);
      }
    }

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(draw_marker);
    array.markers.push_back(lift_marker);
    marker_pub_->publish(array);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  std::vector<SequencePoint> sequence_;
  std::mutex sequence_mutex_;

  std::string student_name_;
  std::string letter_param_;
  std::string planning_group_;
  std::string planning_frame_;

  tf2::Vector3 plane_center_{0, 0, 0};
  tf2::Vector3 raw_u_axis_{0, 1, 0};
  tf2::Vector3 raw_v_axis_{0, 0, 1};
  tf2::Vector3 u_axis_{0, 1, 0};
  tf2::Vector3 v_axis_{0, 0, 1};
  tf2::Vector3 normal_axis_{1, 0, 0};
  geometry_msgs::msg::Quaternion fixed_orientation_;

  double letter_width_ = 0.18;
  double letter_height_ = 0.22;
  double retract_distance_ = 0.05;
  double eef_step_ = 0.005;
  double min_fraction_ = 0.95;
  double velocity_scaling_ = 0.2;
  double acceleration_scaling_ = 0.2;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("letter_writer_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto writer = std::make_shared<LetterWriterNode>(node);
  bool ok = false;

  // Chay logic ve chu tren mot thread rieng, vi cac loi goi MoveGroupInterface
  // (plan/execute) can node duoc spin dong thoi o thread khac.
  std::thread worker([&writer, &ok]() {ok = writer->run();});

  // Spin o thread chinh cho toi khi nguoi dung nhan Ctrl+C (SIGINT) -> nho do
  // marker_timer_ ben trong LetterWriterNode van tiep tuc publish duong di
  // moi giay, RViz them Display luc nao cung thay duoc.
  executor.spin();

  worker.join();
  return ok ? 0 : 1;
}
