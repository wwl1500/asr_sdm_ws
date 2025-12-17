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

using namespace std::chrono_literals;

class AsrSdmControllerNode : public rclcpp::Node
{
public:
  AsrSdmControllerNode() : Node("asr_sdm_controller"), count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Front-Unit-Following Controller");

    // Declare parameters
    this->declare_parameter<double>("segment_length", 0.18);
    this->declare_parameter<int>("num_segments", 4);
    this->declare_parameter<double>("control_frequency", 10.0);
    this->declare_parameter<std::string>("front_unit_velocity_topic", "~/input/front_unit_velocity");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");

    // Get parameters
    double segment_length = this->get_parameter("segment_length").as_double();
    int num_segments = this->get_parameter("num_segments").as_int();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    std::string front_vel_topic = this->get_parameter("front_unit_velocity_topic").as_string();
    std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();

    RCLCPP_INFO(
      this->get_logger(),
      "Controller parameters: segment_length=%.3f, num_segments=%d, frequency=%.1f Hz",
      segment_length, num_segments, control_frequency_);

    // Initialize controller
    try {
      controller_ = std::make_shared<asr_sdm_controller::FrontUnitFollowingController>(
        segment_length, num_segments);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize controller: %s", e.what());
      throw;
    }

    // Publishers
    pub_heartbeat_ =
      this->create_publisher<std_msgs::msg::String>("~/output/controller/heartbeat", 1);
    pub_control_cmd_ =
      this->create_publisher<asr_sdm_control_msgs::msg::ControlCmd>("~/output/control_cmd", 1);

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

    RCLCPP_INFO(this->get_logger(), "Front-Unit-Following Controller initialized");
  }

private:
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

  // ROS 2 interfaces
  rclcpp::TimerBase::SharedPtr timer_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_robot_control_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<asr_sdm_control_msgs::msg::ControlCmd>::SharedPtr pub_control_cmd_;
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
