// Copyright (c) 2025.
// ROS 2 node implementing rigid-body Newton–Euler dynamics using Pinocchio.
//
// Implements the formulas from "Dynamics Modeling of Underwater Multibody
// Robots — Detailed Newton–Euler Derivations" (without fluid forces):
//
//   M_RB(q) * ddq + C_RB(q, dq) * dq + g(q) = tau
//
// Pinocchio API mapping to document equations:
//   Eq. (14)    M_RB(q) = sum_i J_i^T I_i J_i        -> pinocchio::crba()
//   Eqs.(28-34) C_RB(q,dq)*dq + g(q)                 -> pinocchio::nonLinearEffects()
//   Eq. (44)    g_g(q) gravity vector                  -> pinocchio::computeGeneralizedGravity()
//   Eqs.(28-34) C_RB(q,dq) Coriolis matrix            -> pinocchio::computeCoriolisMatrix()
//   Sec.2.7     tau = M*ddq + C*dq + g (N-E recursion)-> pinocchio::rnea()
//   Eqs.(2-3)   J_i(q) geometric Jacobians            -> pinocchio::computeJointJacobians()
//   Eqs.(1,8-9) Forward kinematics (pos/vel/acc)      -> pinocchio::forwardKinematics()
//               Center of mass                         -> pinocchio::centerOfMass()

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

// Pinocchio headers — each maps to a specific set of document equations.
#include <pinocchio/algorithm/center-of-mass.hpp>     // CoM computation
#include <pinocchio/algorithm/compute-all-terms.hpp>   // Efficient combined computation
#include <pinocchio/algorithm/crba.hpp>                // Eq.(14): M_RB via CRBA
#include <pinocchio/algorithm/frames.hpp>              // Frame placement & Jacobians
#include <pinocchio/algorithm/jacobian.hpp>            // Eqs.(2-3): Geometric Jacobians
#include <pinocchio/algorithm/joint-configuration.hpp> // Neutral config, integration
#include <pinocchio/algorithm/kinematics.hpp>          // Eqs.(1,8-9): FK
#include <pinocchio/algorithm/rnea.hpp>                // Sec.2.7 + Eqs.(28-34,44): RNEA, C, g
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace asr_sdm_kinematic_dynamic_model
{

/// Helper: pack an Eigen matrix (row-major) into Float64MultiArray with layout.
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

/// Helper: pack an Eigen vector into Float64MultiArray.
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

// ---------------------------------------------------------------------------
// PinocchioDynamicsNode
// ---------------------------------------------------------------------------
// Computes and publishes all terms of the rigid-body dynamic equation:
//   M_RB(q) * ddq  +  C_RB(q,dq) * dq  +  g(q)  =  tau
//
// Published topics:
//   pinocchio/mass_matrix        — M_RB(q)                    [Eq.(14)]
//   pinocchio/coriolis_matrix    — C_RB(q,dq)                 [Eqs.(28-34)]
//   pinocchio/gravity_vector     — g(q)                       [Eq.(44)]
//   pinocchio/nonlinear_effects  — C_RB*dq + g  (= nle)      [Eqs.(28-34)+(44)]
//   pinocchio/inverse_dynamics   — tau = M*ddq + C*dq + g     [Sec.2.7 RNEA]
//   pinocchio/center_of_mass     — CoM position
//   pinocchio/total_mass         — total robot mass
//   pinocchio/joint_jacobians    — stacked geometric Jacobians [Eqs.(2-3)]
//   pinocchio/link_positions     — link centroid positions (oMi)
//   pinocchio/link_velocities    — link spatial velocities
// ---------------------------------------------------------------------------
class PinocchioDynamicsNode : public rclcpp::Node
{
public:
  PinocchioDynamicsNode() : Node("pinocchio_dynamics_node")
  {
    // ----- Parameters -----
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

    // Desired acceleration for inverse dynamics (default: zero).
    a_ = Eigen::VectorXd::Zero(model_->nv);

    create_publishers();
    create_subscribers();
    configure_timer();
  }

private:
  // -----------------------------------------------------------------------
  // Model loading
  // -----------------------------------------------------------------------
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
        // Free-flyer base provides 6 DOF for the base body in Eqs.(1)-(3).
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

    // Neutral configuration and zero velocity as defaults.
    q_ = pinocchio::neutral(*model_);
    v_ = Eigen::VectorXd::Zero(model_->nv);

    // Optionally seed with user-specified initial joint positions.
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

      // Log joint names for debugging.
      for (int j = 0; j < static_cast<int>(model_->njoints); ++j) {
        RCLCPP_INFO(get_logger(), "  joint[%d]: %s", j, model_->names[j].c_str());
      }
    }
  }

  // -----------------------------------------------------------------------
  // Publishers
  // -----------------------------------------------------------------------
  void create_publishers()
  {
    // M_RB(q) — Eq.(14): rigid-body inertia matrix via CRBA
    mass_matrix_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/mass_matrix", 10);

    // C_RB(q,dq) — Eqs.(28-34): Coriolis-centripetal matrix
    coriolis_matrix_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/coriolis_matrix", 10);

    // g(q) — Eq.(44): generalized gravitational force
    gravity_vector_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/gravity_vector", 10);

    // C_RB(q,dq)*dq + g(q) — combined nonlinear effects
    nle_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/nonlinear_effects", 10);

    // tau = M*ddq + C*dq + g — full inverse dynamics via RNEA (Sec.2.7)
    inverse_dynamics_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/inverse_dynamics", 10);

    // Center of mass position
    com_pub_ =
      create_publisher<geometry_msgs::msg::Vector3>("pinocchio/center_of_mass", 10);

    // Total mass
    total_mass_pub_ =
      create_publisher<std_msgs::msg::Float64>("pinocchio/total_mass", 10);

    // Stacked geometric Jacobians — Eqs.(2-3)
    jacobians_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/joint_jacobians", 10);

    // Link centroid positions & velocities — Eqs.(1,8-9)
    link_positions_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/link_positions", 10);
    link_velocities_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("pinocchio/link_velocities", 10);
  }

  // -----------------------------------------------------------------------
  // Subscriber — /joint_states updates q and v
  // -----------------------------------------------------------------------
  void create_subscribers()
  {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&PinocchioDynamicsNode::joint_state_callback, this, std::placeholders::_1));
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!model_) return;

    // Build a name → index map for the Pinocchio model on first call.
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

      // Position
      if (k < msg->position.size() && nq_j == 1 && q_idx < q_.size()) {
        q_[q_idx] = msg->position[k];
      }
      // Velocity
      if (k < msg->velocity.size() && nv_j == 1 && v_idx < v_.size()) {
        v_[v_idx] = msg->velocity[k];
      }
    }
  }

  // -----------------------------------------------------------------------
  // Timer
  // -----------------------------------------------------------------------
  void configure_timer()
  {
    const int period_ms = get_parameter("publish_period_ms").as_int();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PinocchioDynamicsNode::publish_dynamics, this));
  }

  // -----------------------------------------------------------------------
  // Main computation & publishing loop
  // -----------------------------------------------------------------------
  void publish_dynamics()
  {
    if (!model_ || !data_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Model not ready; ensure URDF path is valid.");
      return;
    }

    // ==================================================================
    // 1. Forward Kinematics — Eqs.(1, 8-9)
    //    Propagates position, velocity, acceleration through all links
    //    using the Newton–Euler recursive forward pass (Sec.2.7).
    // ==================================================================
    pinocchio::forwardKinematics(*model_, *data_, q_, v_, a_);

    // ==================================================================
    // 2. Mass Matrix M_RB(q) — Eq.(14)
    //    M_RB = sum_i J_i^T I_i J_i
    //    CRBA is the Composite Rigid-Body Algorithm that efficiently
    //    computes this sum via a backward recursion.
    // ==================================================================
    pinocchio::crba(*model_, *data_, q_);
    // CRBA only fills the upper triangle; mirror for symmetry.
    data_->M.triangularView<Eigen::StrictlyLower>() = data_->M.transpose();

    mass_matrix_pub_->publish(eigen_to_multiarray(data_->M));

    // ==================================================================
    // 3. Nonlinear Effects: C_RB(q,dq)*dq + g(q) — Eqs.(28-34) + (44)
    //    This is the bias term: what RNEA returns when ddq = 0.
    //    Physically it collects:
    //      - Coriolis / centripetal forces (omega_i × J_i omega_i)
    //      - Gravitational generalized force
    // ==================================================================
    pinocchio::nonLinearEffects(*model_, *data_, q_, v_);

    nle_pub_->publish(eigen_to_multiarray(data_->nle, "nle"));

    // ==================================================================
    // 4. Gravity Vector g(q) — Eq.(44)
    //    g_g(q) = sum_i J_i^T [ R_b^i [0,0,-m_i g]^T ; ... ]
    // ==================================================================
    pinocchio::computeGeneralizedGravity(*model_, *data_, q_);

    gravity_vector_pub_->publish(eigen_to_multiarray(data_->g, "gravity"));

    // ==================================================================
    // 5. Coriolis Matrix C_RB(q,dq) — Eqs.(28-34)
    //    Constructed so that (Mdot - 2C) is skew-symmetric (Eq.31).
    //    Uses Christoffel symbols: C_ij = sum_k Gamma_ijk dq_k  (Eq.29)
    // ==================================================================
    pinocchio::computeCoriolisMatrix(*model_, *data_, q_, v_);

    coriolis_matrix_pub_->publish(eigen_to_multiarray(data_->C));

    // ==================================================================
    // 6. Inverse Dynamics via RNEA — Sec.2.7 (Newton–Euler Recursive)
    //    tau = M(q)*ddq + C(q,dq)*dq + g(q)
    //
    //    The RNEA performs:
    //      Forward pass (i = 0..n):
    //        omega_{i+1} = R^T omega_i + dtheta z    [angular vel propagation]
    //        domega_{i+1} = ...                       [angular accel propagation]
    //        a_{c,i+1} = ...                          [linear accel, Eq. in Sec.2]
    //        F_i = m_i a_{c,i}                        [Newton, Eq.(Newton)]
    //        N_i = I_i domega_i + omega_i × I_i omega_i [Euler, Eq.(Euler)]
    //      Backward pass (i = n..0):
    //        f_i = R f_{i+1} + F_i                    [force transmission]
    //        n_i = N_i + R n_{i+1} + ...              [moment transmission]
    //        tau_i = n_i^T z_i                         [joint torque extraction]
    // ==================================================================
    const Eigen::VectorXd tau = pinocchio::rnea(*model_, *data_, q_, v_, a_);

    inverse_dynamics_pub_->publish(eigen_to_multiarray(tau, "tau"));

    // ==================================================================
    // 7. Geometric Jacobians — Eqs.(2-3)
    //    J_i = [ J_{i,b}  J_{i,theta} ]
    //    Each column j: [ z_j × p_{i/j} ; z_j ] (revolute)
    // ==================================================================
    pinocchio::computeJointJacobians(*model_, *data_, q_);

    {
      // Stack all joint Jacobians into a single (njoints-1)*6 × nv matrix.
      const int n_active_joints = model_->njoints - 1;  // exclude "universe"
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

    // ==================================================================
    // 8. Link Positions & Velocities — Forward kinematics results
    //    data_.oMi[j].translation() = position of joint j origin in world
    //    data_.v[j]                 = spatial velocity of joint j
    // ==================================================================
    {
      const int n_active_joints = model_->njoints - 1;
      // Positions: (n_active_joints × 3)
      Eigen::MatrixXd positions(n_active_joints, 3);
      // Velocities: (n_active_joints × 6) — [linear; angular]
      Eigen::MatrixXd velocities(n_active_joints, 6);

      for (int j = 1; j < model_->njoints; ++j) {
        positions.row(j - 1) = data_->oMi[j].translation().transpose();
        velocities.row(j - 1).head(3) = data_->v[j].linear().transpose();
        velocities.row(j - 1).tail(3) = data_->v[j].angular().transpose();
      }
      link_positions_pub_->publish(eigen_to_multiarray(positions, "links", "xyz"));
      link_velocities_pub_->publish(eigen_to_multiarray(velocities, "links", "lin_ang"));
    }

    // ==================================================================
    // 9. Center of Mass
    // ==================================================================
    const Eigen::Vector3d com = pinocchio::centerOfMass(*model_, *data_, q_, v_);

    geometry_msgs::msg::Vector3 com_msg;
    com_msg.x = com.x();
    com_msg.y = com.y();
    com_msg.z = com.z();
    com_pub_->publish(com_msg);

    // ==================================================================
    // 10. Total Mass
    // ==================================================================
    const double total_mass = pinocchio::computeTotalMass(*model_, *data_);

    std_msgs::msg::Float64 mass_msg;
    mass_msg.data = total_mass;
    total_mass_pub_->publish(mass_msg);
  }

  // -----------------------------------------------------------------------
  // Member variables
  // -----------------------------------------------------------------------

  // Publishers
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

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Pinocchio model & data
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;

  // State vectors:  q ∈ R^nq (configuration),  v ∈ R^nv (velocity),  a ∈ R^nv (acceleration)
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd a_;  // desired/commanded acceleration for inverse dynamics

  // Joint name lookup (built lazily from first JointState message)
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
