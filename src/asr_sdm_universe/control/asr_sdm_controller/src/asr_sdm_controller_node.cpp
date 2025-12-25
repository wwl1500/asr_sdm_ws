/* standard headers */
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <map>

/* ROS2 headers */
#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_control_msgs/msg/control_cmd.hpp"
#include "asr_sdm_control_msgs/msg/unit_cmd.hpp"
#include "asr_sdm_hardware_msgs/msg/can_frame.hpp"
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "asr_sdm_controller/front_unit_following_controller.hpp"
#include "asr_sdm_controller/pinocchio_kinematics_wrapper.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class AsrSdmControllerNode : public rclcpp::Node
{
public:
  AsrSdmControllerNode() : Node("asr_sdm_controller"), count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "初始化前端单元跟随控制器");

    // 声明参数
    this->declare_parameter<double>("segment_length", 0.18);
    this->declare_parameter<int>("num_segments", 4);
    this->declare_parameter<double>("control_frequency", 10.0);
    this->declare_parameter<std::string>("front_unit_velocity_topic", "~/input/front_unit_velocity");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    
    // Pinocchio 参数
    this->declare_parameter<bool>("use_pinocchio", true);
    this->declare_parameter<std::string>("urdf_param_name", "robot_description");
    this->declare_parameter<bool>("fallback_on_error", true);
    
    // 诊断参数
    this->declare_parameter<bool>("enable_diagnostics", true);
    this->declare_parameter<double>("diagnostics_frequency", 1.0);
    this->declare_parameter<double>("velocity_warning_threshold", 2.0);

    // 获取参数
    double segment_length = this->get_parameter("segment_length").as_double();
    int num_segments = this->get_parameter("num_segments").as_int();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    std::string front_vel_topic = this->get_parameter("front_unit_velocity_topic").as_string();
    std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();
    
    use_pinocchio_ = this->get_parameter("use_pinocchio").as_bool();
    std::string urdf_param_name = this->get_parameter("urdf_param_name").as_string();
    fallback_on_error_ = this->get_parameter("fallback_on_error").as_bool();
    enable_diagnostics_ = this->get_parameter("enable_diagnostics").as_bool();
    velocity_warning_threshold_ = this->get_parameter("velocity_warning_threshold").as_double();

    RCLCPP_INFO(
      this->get_logger(),
      "控制器参数: segment_length=%.3f, num_segments=%d, frequency=%.1f Hz, use_pinocchio=%s",
      segment_length, num_segments, control_frequency_, use_pinocchio_ ? "true" : "false");

    // 初始化 Pinocchio（如果启用）
    std::shared_ptr<asr_sdm_controller::PinocchioKinematicsWrapper> pinocchio_wrapper = nullptr;
    if (use_pinocchio_) {
      pinocchio_wrapper = initializePinocchio(urdf_param_name);
    }

    // 初始化控制器
    try {
      if (pinocchio_wrapper && pinocchio_wrapper->isInitialized()) {
        controller_ = std::make_shared<asr_sdm_controller::FrontUnitFollowingController>(
          segment_length, num_segments, pinocchio_wrapper);
        RCLCPP_INFO(this->get_logger(), "控制器已使用 Pinocchio 初始化");
      } else {
        controller_ = std::make_shared<asr_sdm_controller::FrontUnitFollowingController>(
          segment_length, num_segments);
        RCLCPP_INFO(this->get_logger(), "控制器已使用简化模型初始化");
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "初始化控制器失败: %s", e.what());
      throw;
    }

    // Publishers
    pub_heartbeat_ =
      this->create_publisher<std_msgs::msg::String>("~/output/controller/heartbeat", 1);
    pub_control_cmd_ =
      this->create_publisher<asr_sdm_control_msgs::msg::ControlCmd>("~/output/control_cmd", 1);
    
    // 诊断发布器
    if (enable_diagnostics_) {
      pub_diagnostics_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 1);
      pub_computation_method_ =
        this->create_publisher<std_msgs::msg::String>("~/computation_method", 1);
      pub_computation_time_ =
        this->create_publisher<std_msgs::msg::Float64>("~/computation_time", 1);
    }

    // Subscribers
    sub_front_unit_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
      front_vel_topic, 10,
      std::bind(&AsrSdmControllerNode::frontUnitVelocityCallback, this, std::placeholders::_1));

    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10,
      std::bind(&AsrSdmControllerNode::jointStateCallback, this, std::placeholders::_1));

    // Timers
    timer_heartbeat_ =
      this->create_wall_timer(1500ms, std::bind(&AsrSdmControllerNode::timer_heartbeat, this));

    auto control_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_));
    timer_robot_control_ =
      this->create_wall_timer(control_period, std::bind(&AsrSdmControllerNode::timer_controller, this));

    // Initialize state
    front_unit_velocity_received_ = false;
    joint_state_received_ = false;
    current_v1_ = 0.0;
    current_omega1_ = 0.0;
    
    // 诊断定时器
    if (enable_diagnostics_) {
      double diag_freq = this->get_parameter("diagnostics_frequency").as_double();
      auto diag_period = std::chrono::milliseconds(static_cast<int>(1000.0 / diag_freq));
      timer_diagnostics_ =
        this->create_wall_timer(diag_period, std::bind(&AsrSdmControllerNode::publishDiagnostics, this));
    }

    RCLCPP_INFO(this->get_logger(), "前端单元跟随控制器初始化完成");
  }

private:
  /**
   * @brief 初始化 Pinocchio
   * @param urdf_param_name URDF 参数名称
   * @return Pinocchio 封装器指针
   */
  std::shared_ptr<asr_sdm_controller::PinocchioKinematicsWrapper> initializePinocchio(
    const std::string & urdf_param_name)
  {
    try {
      // 尝试从参数服务器获取 URDF
      // 注意：这需要 robot_state_publisher 先运行
      RCLCPP_INFO(this->get_logger(), "尝试从参数 '%s' 加载 URDF", urdf_param_name.c_str());
      
      // 创建参数客户端获取 robot_description
      auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
        this, "/robot_state_publisher");
      
      if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(this->get_logger(), "robot_state_publisher 服务不可用，使用简化模型");
        return nullptr;
      }
      
      auto parameters = parameters_client->get_parameters({urdf_param_name});
      if (parameters.empty()) {
        RCLCPP_WARN(this->get_logger(), "未找到参数 '%s'，使用简化模型", urdf_param_name.c_str());
        return nullptr;
      }
      
      std::string urdf_string = parameters[0].as_string();
      if (urdf_string.empty()) {
        RCLCPP_WARN(this->get_logger(), "URDF 字符串为空，使用简化模型");
        return nullptr;
      }
      
      auto wrapper = std::make_shared<asr_sdm_controller::PinocchioKinematicsWrapper>();
      if (wrapper->initializeFromUrdfString(urdf_string)) {
        RCLCPP_INFO(this->get_logger(), "Pinocchio 初始化成功");
        return wrapper;
      } else {
        RCLCPP_WARN(this->get_logger(), "Pinocchio 初始化失败，使用简化模型");
        return nullptr;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "初始化 Pinocchio 时发生异常: %s，使用简化模型", e.what());
      return nullptr;
    }
  }
  
  /**
   * @brief 发布诊断信息
   */
  void publishDiagnostics()
  {
    if (!enable_diagnostics_) return;
    
    // 发布计算方法
    auto method_msg = std_msgs::msg::String();
    method_msg.data = controller_->getComputationMethod();
    pub_computation_method_->publish(method_msg);
    
    // 发布诊断数组
    auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
    diag_msg.header.stamp = this->now();
    
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "FrontUnitFollowingController";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "控制器运行正常";
    
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "computation_method";
    kv.value = controller_->getComputationMethod();
    status.values.push_back(kv);
    
    kv.key = "pinocchio_enabled";
    kv.value = controller_->isPinocchioEnabled() ? "true" : "false";
    status.values.push_back(kv);
    
    diag_msg.status.push_back(status);
    pub_diagnostics_->publish(diag_msg);
  }

  void timer_heartbeat()
  {
    RCLCPP_INFO(this->get_logger(), "Control Node Heartbeat");
  }

  void frontUnitVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Extract v1 (linear velocity in x direction) and omega1 (angular velocity in z)
    current_v1_ = msg->linear.x;
    current_omega1_ = msg->angular.z;
    front_unit_velocity_received_ = true;

    RCLCPP_DEBUG(
      this->get_logger(),
      "Received front unit velocity: v1=%.3f, omega1=%.3f",
      current_v1_, current_omega1_);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Extract joint angles from joint state
    // Joint names should match: base_link_to_link_2, link_2_to_link_3, etc.
    current_joint_angles_.clear();
    joint_name_to_index_.clear();

    for (size_t i = 0; i < msg->name.size(); ++i) {
      std::string joint_name = msg->name[i];
      // Check if this is a link-to-link joint (not screw joints)
      if (joint_name.find("_to_link_") != std::string::npos ||
        joint_name.find("base_link_to_link_") != std::string::npos)
      {
        joint_name_to_index_[joint_name] = i;
        if (i < msg->position.size()) {
          current_joint_angles_.push_back(msg->position[i]);
        }
      }
    }

    // Sort joints by name to ensure correct order
    std::vector<std::string> sorted_joint_names;
    for (const auto & pair : joint_name_to_index_) {
      sorted_joint_names.push_back(pair.first);
    }
    std::sort(sorted_joint_names.begin(), sorted_joint_names.end());

    current_joint_angles_.clear();
    for (const auto & name : sorted_joint_names) {
      int idx = joint_name_to_index_[name];
      if (idx < static_cast<int>(msg->position.size())) {
        current_joint_angles_.push_back(msg->position[idx]);
      }
    }

    joint_state_received_ = true;

    RCLCPP_DEBUG(
      this->get_logger(),
      "Received joint state: %zu joints",
      current_joint_angles_.size());
  }

  void timer_controller()
  {
    // Check if we have all required data
    if (!front_unit_velocity_received_ || !joint_state_received_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Waiting for data: velocity=%d, joint_state=%d",
        front_unit_velocity_received_, joint_state_received_);
      return;
    }

    // Check if we have enough joint angles
    int num_segments = controller_->getNumSegments();
    if (static_cast<int>(current_joint_angles_.size()) < num_segments) {
      RCLCPP_WARN(
        this->get_logger(),
        "Not enough joint angles: have %zu, need %d",
        current_joint_angles_.size(), num_segments);
      return;
    }

    try {
      // Compute joint angular velocities using the controller
      std::vector<double> joint_velocities = controller_->computeJointVelocities(
        current_v1_,
        current_omega1_,
        current_joint_angles_);

      // Convert to control command
      auto control_cmd = asr_sdm_control_msgs::msg::ControlCmd();
      control_cmd.header.stamp = this->now();
      control_cmd.header.frame_id = "base_link";

      // Create unit commands for each segment
      // Note: This mapping may need adjustment based on actual hardware configuration
      for (int i = 0; i < num_segments; ++i) {
        asr_sdm_control_msgs::msg::UnitCmd unit_cmd;
        unit_cmd.unit_id = i + 1;  // Unit IDs start from 1

        // Convert joint angular velocity to joint angle (simple integration)
        // Use the control period as the time step
        double dt = 1.0 / control_frequency_;
        double target_angle = current_joint_angles_[i] + joint_velocities[i] * dt;

        // Clamp angle to reasonable range
        target_angle = std::max(-3.14159, std::min(3.14159, target_angle));

        // Set joint angles (convert from radians to integer if needed)
        // Assuming the hardware expects integer values, scale appropriately
        unit_cmd.joint1_angle = static_cast<int32_t>(target_angle * 1000.0);  // Scale factor
        unit_cmd.joint2_angle = 0;  // Second joint if applicable

        // Screw velocities are not computed by this controller
        // They should be set separately or left at 0
        unit_cmd.screw1_vel = 0;
        unit_cmd.screw2_vel = 0;

        control_cmd.units_cmd.push_back(unit_cmd);
      }

      // Publish control command
      pub_control_cmd_->publish(control_cmd);

      RCLCPP_DEBUG(
        this->get_logger(),
        "Published control command with %zu units",
        control_cmd.units_cmd.size());

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error computing control: %s", e.what());
    }
  }

  size_t count_;

  // Controller
  std::shared_ptr<asr_sdm_controller::FrontUnitFollowingController> controller_;

    // State
    bool front_unit_velocity_received_;
    bool joint_state_received_;
    double current_v1_;
    double current_omega1_;
    double control_frequency_;
    std::vector<double> current_joint_angles_;
    std::map<std::string, int> joint_name_to_index_;
    
    // Pinocchio 相关
    bool use_pinocchio_;
    bool fallback_on_error_;
    bool enable_diagnostics_;
    double velocity_warning_threshold_;

  // ROS 2 interfaces
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_robot_control_;
  rclcpp::TimerBase::SharedPtr timer_diagnostics_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::ControlCmd>::SharedPtr pub_control_cmd_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_computation_method_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_computation_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_front_unit_velocity_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AsrSdmControllerNode>());
  rclcpp::shutdown();
  return 0;
}
