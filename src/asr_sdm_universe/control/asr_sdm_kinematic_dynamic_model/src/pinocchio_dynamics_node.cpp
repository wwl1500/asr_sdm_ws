// Copyright (c) 2025.
// 使用 Pinocchio 实现刚体牛顿-欧拉动力学的 ROS 2 节点。
// 核心方程：tau = M(q) * ddq + C(q, dq) * dq + g(q)。
// 节点从 /joint_states 接收 (q, v)，周期发布 M、C、g、nle、tau、J、连杆状态与质心。

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Dense>

// Pinocchio 算法接口。
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace asr_sdm_kinematic_dynamic_model
{

/// 将 Eigen 矩阵按行展平为 Float64MultiArray，并写入二维 layout。
/// 输入：任意 m x n 矩阵。
/// 输出：data 长度为 m*n，layout.dim 分别表示行与列。
static std_msgs::msg::Float64MultiArray eigen_to_multiarray(
  const Eigen::MatrixXd & mat, const std::string & row_label = "rows",
  const std::string & col_label = "cols")
{
  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.resize(2);
  msg.layout.dim[0].label  = row_label;
  msg.layout.dim[0].size   = static_cast<uint32_t>(mat.rows());
  msg.layout.dim[0].stride = static_cast<uint32_t>(mat.cols());
  msg.layout.dim[1].label  = col_label;
  msg.layout.dim[1].size   = static_cast<uint32_t>(mat.cols());
  msg.layout.dim[1].stride = 1;
  msg.data.reserve(static_cast<size_t>(mat.rows() * mat.cols()));
  for (Eigen::Index r = 0; r < mat.rows(); ++r) {
    for (Eigen::Index c = 0; c < mat.cols(); ++c) {
      msg.data.push_back(mat(r, c));
    }
  }
  return msg;
}

/// 将 Eigen 向量转换为一维 Float64MultiArray。
/// 输入：长度为 n 的向量。
/// 输出：layout.dim[0].size = n，data 顺序与 Eigen 向量一致。
static std_msgs::msg::Float64MultiArray eigen_to_multiarray(
  const Eigen::VectorXd & vec, const std::string & label = "dim")
{
  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label  = label;
  msg.layout.dim[0].size   = static_cast<uint32_t>(vec.size());
  msg.layout.dim[0].stride = 1;
  msg.data.assign(vec.data(), vec.data() + vec.size());
  return msg;
}

/// 动力学计算节点：加载 URDF 后周期计算并发布 Pinocchio 动力学/运动学结果。
class PinocchioDynamicsNode : public rclcpp::Node
{
public:
  /// 构造函数：声明参数、加载模型、初始化状态并创建 ROS 接口。
  PinocchioDynamicsNode() : Node("pinocchio_dynamics_node")
  {
    declare_parameter<std::string>("robot_description_path", "");
    declare_parameter<std::string>("robot_description_package", "asr_sdm_description");
    declare_parameter<std::string>("robot_description_file", "urdf/underwater_snakerobot.urdf");
    declare_parameter<bool>("use_free_flyer", true);
    declare_parameter<bool>("log_model_summary", true);
    declare_parameter<int>("publish_period_ms", 200);
    declare_parameter<std::vector<double>>("initial_joint_positions", std::vector<double>{});

    load_robot_model();
    if (!model_) {
      throw std::runtime_error("Failed to create Pinocchio model.");
    }

    a_ = Eigen::VectorXd::Zero(model_->nv);

    create_publishers();
    create_subscribers();
    configure_timer();
  }

private:
  /// 加载 URDF 并构建 Pinocchio 模型与数据缓存。
  /// 流程：解析参数 -> 解析路径 -> buildModel -> 初始化 q_/v_ -> 可选打印模型摘要。
  void load_robot_model()
  {
    const auto explicit_path  = get_parameter("robot_description_path").as_string();
    const auto package_name   = get_parameter("robot_description_package").as_string();
    const auto relative_path  = get_parameter("robot_description_file").as_string();
    const bool use_free_flyer = get_parameter("use_free_flyer").as_bool();

    std::string resolved_urdf;
    if (!explicit_path.empty()) {
      resolved_urdf = explicit_path;
    } else {
      try {
        const auto share = ament_index_cpp::get_package_share_directory(package_name);
        resolved_urdf = share + "/" + relative_path;
      } catch (const std::exception & e) {
        RCLCPP_FATAL(get_logger(), "Package '%s' not found: %s", package_name.c_str(), e.what());
        return;
      }
    }

    if (!std::ifstream(resolved_urdf)) {
      RCLCPP_FATAL(get_logger(), "URDF not found: %s", resolved_urdf.c_str());
      return;
    }

    model_ = std::make_shared<pinocchio::Model>();
    try {
      if (use_free_flyer) {
        pinocchio::urdf::buildModel(resolved_urdf, pinocchio::JointModelFreeFlyer(), *model_);
      } else {
        pinocchio::urdf::buildModel(resolved_urdf, *model_);
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Pinocchio build failed: %s", e.what());
      model_.reset();
      return;
    }

    data_ = std::make_shared<pinocchio::Data>(*model_);

    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(model_->nv);

    std::vector<double> init_pos;
    get_parameter("initial_joint_positions", init_pos);
    if (!init_pos.empty()) {
      const auto len = static_cast<int>(
        std::min(init_pos.size(), static_cast<size_t>(q_.size())));
      for (int i = 0; i < len; ++i) {
        q_[i] = init_pos[i];
      }
      if (static_cast<size_t>(q_.size()) != init_pos.size()) {
        RCLCPP_WARN(get_logger(),
          "initial_joint_positions size (%zu) != model nq (%ld); remaining joints keep neutral.",
          init_pos.size(), static_cast<long>(q_.size()));
      }
    }

    if (get_parameter("log_model_summary").as_bool()) {
      RCLCPP_INFO(get_logger(),
        "Model: %s | nq=%d nv=%d | joints=%d frames=%d",
        model_->name.c_str(), model_->nq, model_->nv,
        static_cast<int>(model_->njoints),
        model_->nframes);

      for (int j = 0; j < static_cast<int>(model_->njoints); ++j) {
        RCLCPP_INFO(get_logger(), "  joint[%d]: %s", j, model_->names[j].c_str());
      }
    }
  }

  /// 创建所有发布器。
  /// 话题包含：M、C、g、nle、tau、质心、总质量、堆叠雅可比、连杆位置和连杆速度。
  void create_publishers()
  {
    mass_matrix_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/mass_matrix", 10);

    coriolis_matrix_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/coriolis_matrix", 10);

    gravity_vector_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/gravity_vector", 10);

    nle_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/nonlinear_effects", 10);

    inverse_dynamics_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/inverse_dynamics", 10);

    com_pub_ =
      create_publisher<geometry_msgs::msg::Vector3>("pinocchio/center_of_mass", 10);

    total_mass_pub_ =
      create_publisher<std_msgs::msg::Float64>("pinocchio/total_mass", 10);

    jacobians_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/joint_jacobians", 10);

    link_positions_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/link_positions", 10);
    link_velocities_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/link_velocities", 10);
  }

  /// 创建订阅器：从 /joint_states 接收状态并更新 q_/v_。
  void create_subscribers()
  {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&PinocchioDynamicsNode::joint_state_callback, this, std::placeholders::_1));
  }

  /// 处理 JointState：按关节名映射更新模型状态。
  /// 当前仅写入标量关节（nq==1 且 nv==1），多自由度关节保持原值。
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!model_) return;

    if (joint_name_to_model_idx_.empty()) {
      for (int j = 1; j < static_cast<int>(model_->njoints); ++j) {
        joint_name_to_model_idx_[model_->names[j]] = j;
      }
    }

    for (size_t k = 0; k < msg->name.size(); ++k) {
      auto it = joint_name_to_model_idx_.find(msg->name[k]);
      if (it == joint_name_to_model_idx_.end()) continue;

      const int joint_id = it->second;
      const int q_idx    = model_->joints[joint_id].idx_q();
      const int v_idx    = model_->joints[joint_id].idx_v();
      const int nq_j     = model_->joints[joint_id].nq();
      const int nv_j     = model_->joints[joint_id].nv();

      if (k < msg->position.size() && nq_j == 1 && q_idx < q_.size()) {
        q_[q_idx] = msg->position[k];
      }
      if (k < msg->velocity.size() && nv_j == 1 && v_idx < v_.size()) {
        v_[v_idx] = msg->velocity[k];
      }
    }
  }

  /// 配置周期定时器，驱动动力学计算与发布。
  void configure_timer()
  {
    const int period_ms = get_parameter("publish_period_ms").as_int();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PinocchioDynamicsNode::publish_dynamics, this));
  }

  /// 主循环：基于当前 q_/v_/a_ 计算并发布动力学与运动学结果。
  /// 计算顺序：前向运动学 -> M -> nle -> g -> C -> tau -> J -> 连杆状态 -> CoM -> 总质量。
  void publish_dynamics()
  {
    if (!model_ || !data_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Model not ready; ensure URDF path is valid.");
      return;
    }

    pinocchio::forwardKinematics(*model_, *data_, q_, v_, a_);

    pinocchio::crba(*model_, *data_, q_);
    data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose();
    mass_matrix_pub_->publish(eigen_to_multiarray(data_->M));

    pinocchio::nonLinearEffects(*model_, *data_, q_, v_);
    nle_pub_->publish(eigen_to_multiarray(data_->nle, "nle"));

    pinocchio::computeGeneralizedGravity(*model_, *data_, q_);
    gravity_vector_pub_->publish(eigen_to_multiarray(data_->g, "gravity"));

    pinocchio::computeCoriolisMatrix(*model_, *data_, q_, v_);
    coriolis_matrix_pub_->publish(eigen_to_multiarray(data_->C));

    const Eigen::VectorXd tau = pinocchio::rnea(*model_, *data_, q_, v_, a_);
    inverse_dynamics_pub_->publish(eigen_to_multiarray(tau, "tau"));

    pinocchio::computeJointJacobians(*model_, *data_, q_);
    {
      const int n_active_joints = model_->njoints - 1;
      Eigen::MatrixXd J_stacked(6 * n_active_joints, model_->nv);
      J_stacked.setZero();
      for (int j = 1; j < model_->njoints; ++j) {
        Eigen::MatrixXd J_j(6, model_->nv);
        J_j.setZero();
        pinocchio::getJointJacobian(*model_, *data_, j,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J_j);
        J_stacked.middleRows(6 * (j - 1), 6) = J_j;
      }
      jacobians_pub_->publish(eigen_to_multiarray(J_stacked, "J_rows", "J_cols"));
    }

    {
      const int n_active_joints = model_->njoints - 1;
      Eigen::MatrixXd positions(n_active_joints, 3);
      Eigen::MatrixXd velocities(n_active_joints, 6);

      for (int j = 1; j < model_->njoints; ++j) {
        positions.row(j - 1) = data_->oMi[j].translation().transpose();
        velocities.row(j - 1).head(3) = data_->v[j].linear().transpose();
        velocities.row(j - 1).tail(3) = data_->v[j].angular().transpose();
      }
      link_positions_pub_->publish(eigen_to_multiarray(positions, "links", "xyz"));
      link_velocities_pub_->publish(eigen_to_multiarray(velocities, "links", "lin_ang"));
    }

    const Eigen::Vector3d com = pinocchio::centerOfMass(*model_, *data_, q_, v_);
    geometry_msgs::msg::Vector3 com_msg;
    com_msg.x = com.x();
    com_msg.y = com.y();
    com_msg.z = com.z();
    com_pub_->publish(com_msg);

    const double total_mass = pinocchio::computeTotalMass(*model_, *data_);
    std_msgs::msg::Float64 mass_msg;
    mass_msg.data = total_mass;
    total_mass_pub_->publish(mass_msg);
  }

  // 发布器
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mass_matrix_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr coriolis_matrix_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_vector_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr nle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr inverse_dynamics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr com_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_mass_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobians_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr link_positions_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr link_velocities_pub_;

  // 订阅器
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // Pinocchio 模型与运行时数据缓存。
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;

  // 状态向量：q（构型）、v（速度）、a（加速度）。
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd a_;

  // 关节名到模型索引的映射（首次回调时懒加载）。
  std::unordered_map<std::string, int> joint_name_to_model_idx_;
};

}  // namespace asr_sdm_kinematic_dynamic_model

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<asr_sdm_kinematic_dynamic_model::PinocchioDynamicsNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("pinocchio_dynamics_node"), "Unhandled exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
